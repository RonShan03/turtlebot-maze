"""Independent Zenoh RGB-D -> YOLO/CLIP -> semantic database implementation."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import queue
import time

import cv2
import numpy as np

from .database import DatabaseStore, connect_database
from .detector_pipeline import accept_keyframe, process_keyframe
from .geometry import (_timestamp_ns, backproject_pixel_to_map, median_depth_in_bbox,
                       pose_to_transform, resolve_map_pose)
from .messages import Image, Odometry, TFMessage
from .synchronization import TimestampBuffer, image_array, yaw


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--connect', default='tcp/localhost:7447')
    p.add_argument('--model', default='yolov8n.pt')
    p.add_argument('--duration', type=float, default=0, help='Wall seconds; zero runs until Ctrl+C')
    p.add_argument('--output', required=True, type=Path, help='New run directory')
    p.add_argument('--schema', required=True, help='Use a fresh schema per independent experiment')
    p.add_argument('--radius', type=float, default=0.5)
    p.add_argument('--map-to-odom', nargs=3, type=float, metavar=('X','Y','YAW'),
                   help='Explicit static T_map_odom, radians; default uses timestamped /tf')
    p.add_argument('--pg-host', default=os.getenv('PGHOST','localhost'))
    p.add_argument('--pg-port', type=int, default=int(os.getenv('PGPORT','5436')))
    p.add_argument('--pg-db', default=os.getenv('PGDATABASE','vectordb'))
    return p


def main():
    args = parser().parse_args()
    if args.radius <= 0:
        raise ValueError('Radius must be positive')
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'frames').mkdir()
    metadata = dict(vars(args), output=str(args.output), started_utc=datetime.now(timezone.utc).isoformat(),
                    confidence=0.3, clip='ViT-B-32/laion2b_s34b_b79k', max_hz=10,
                    translation_m=0.5, rotation_deg=15, sync_tolerance_ms=200)
    (args.output/'run.json').write_text(json.dumps(metadata,indent=2))
    import torch
    import open_clip
    import zenoh
    from ultralytics import YOLO
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    torch.set_num_threads(min(4, os.cpu_count() or 1))
    model = YOLO(args.model)
    clip, _, preprocess = open_clip.create_model_and_transforms('ViT-B-32', pretrained='laion2b_s34b_b79k')
    clip = clip.to(device).eval()
    conn = connect_database(args.schema, host=args.pg_host,port=args.pg_port,dbname=args.pg_db,
                            user=os.getenv('PGUSER','postgres'),password=os.getenv('PGPASSWORD','postgres'))
    store = DatabaseStore(conn)
    incoming = queue.Queue(maxsize=600)
    counters = Counter()
    def receive(kind, cls):
        def callback(sample):
            try:
                incoming.put_nowait((kind,cls.deserialize(bytes(sample.payload)),time.monotonic()))
            except queue.Full:
                counters['queue_dropped'] += 1
            except Exception as exc:
                counters['decode_errors'] += 1
                print(f'{kind} decode error: {exc}', flush=True)
        return callback
    config = zenoh.Config()
    config.insert_json5('mode',json.dumps('client'))
    config.insert_json5('scouting/multicast/enabled','false')
    config.insert_json5('connect/endpoints',json.dumps([args.connect]))
    session = zenoh.open(config)
    subscribers = [session.declare_subscriber(key,receive(kind,cls)) for key,kind,cls in [
        ('camera/color/image_raw','color',Image),('camera/depth/image_rect_raw','depth',Image),
        ('odom','odom',Odometry),('tf','tf',TFMessage),('tf_static','tf',TFMessage)]]
    publisher = session.declare_publisher('tb/detections')
    depth, odom, transforms = TimestampBuffer(), TimestampBuffer(), TimestampBuffer(600)
    pending = []
    previous_pose = None
    last_color_stamp = None
    keyframe_id = 0
    started = time.monotonic()
    last_status = started
    print(f'RUNNING {args.schema} device={device}',flush=True)
    try:
        with (args.output/'keyframes.jsonl').open('w',encoding='utf-8') as log, \
             (args.output/'route.jsonl').open('w',encoding='utf-8') as route:
            while not args.duration or time.monotonic()-started < args.duration:
                try:
                    kind,msg,arrival = incoming.get(timeout=0.05)
                    counters[kind+'_received'] += 1
                    if kind == 'depth': depth.add(msg)
                    elif kind == 'odom': odom.add(msg)
                    elif kind == 'tf':
                        for tf in msg.transforms:
                            if tf.header.frame_id.lstrip('/') == 'map' and tf.child_frame_id.lstrip('/') == 'odom':
                                transforms.add(tf)
                    else:
                        stamp = _timestamp_ns(msg)
                        if last_color_stamp is None or stamp-last_color_stamp >= 100_000_000:
                            pending.append((msg,arrival))
                            last_color_stamp = stamp
                except queue.Empty:
                    pass
                # Drain input before inference; keep bounded histories across delayed callbacks.
                if not incoming.empty():
                    continue
                while pending and time.monotonic()-pending[0][1] >= 0.25:
                    color,_ = pending.pop(0)
                    stamp = _timestamp_ns(color)
                    d, o = depth.nearest(stamp), odom.nearest(stamp)
                    if d is None or o is None:
                        counters['unsynchronized'] += 1
                        continue
                    tf = transforms.nearest(stamp,2_000_000_000)
                    map_to_odom = args.map_to_odom
                    if map_to_odom is None and tf is not None:
                        t = tf.transform.translation
                        map_to_odom = (t.x,t.y,yaw(tf.transform.rotation))
                    p = o.pose.pose.position
                    try:
                        pose = resolve_map_pose((p.x,p.y,yaw(o.pose.pose.orientation)),o.header.frame_id,map_to_odom)
                    except ValueError:
                        counters['missing_map_transform'] += 1
                        continue
                    route.write(json.dumps(dict(stamp_ns=stamp,x=pose[0],y=pose[1],yaw=pose[2]))+'\n')
                    if not accept_keyframe(pose,previous_pose):
                        counters['motion_rejected'] += 1
                        continue
                    frame, depth_m = image_array(color), image_array(d)
                    if frame.shape[:2] != (240,320) or depth_m.shape != (240,320):
                        raise ValueError('Camera dimensions differ from assignment calibration')
                    detections = process_keyframe(frame,model,0.3,clip,preprocess,device,model.names)
                    keyframe_id += 1
                    previous_pose = pose
                    camera = (pose_to_transform(*pose) @ np.array([0.064,-0.065,0.094,1]))[:3]
                    camera_pose = [*camera.tolist(),0.,0.,math.sin(pose[2]/2),math.cos(pose[2]/2)]
                    envelope = dict(keyframe_id=keyframe_id,stamp_ns=stamp,map_x=pose[0],map_y=pose[1],
                                    map_yaw=pose[2],depth_stamp_ns=_timestamp_ns(d),odom_stamp_ns=_timestamp_ns(o),
                                    map_to_odom=map_to_odom,detections=[])
                    annotated = frame.copy()
                    for index,det in enumerate(detections):
                        record = det.as_dict()
                        record['detection_index'] = index
                        x1,y1,x2,y2 = det.bbox
                        distance = median_depth_in_bbox(depth_m,det.bbox)
                        record['median_depth_m'] = distance
                        if distance is not None and det.embedding is not None:
                            position = backproject_pixel_to_map((x1+x2)/2,(y1+y2)/2,distance,pose)
                            dx,dy = position[0]-pose[0],position[1]-pose[1]
                            bearing = math.atan2(math.sin(math.atan2(dy,dx)-pose[2]), math.cos(math.atan2(dy,dx)-pose[2]))
                            object_id,observation_id = store.ingest(det.class_name,position,det.embedding,stamp,
                                pose,camera_pose,det.bbox,det.confidence,bearing,math.hypot(dx,dy),args.radius)
                            record.update(position=position.tolist(),object_id=object_id,observation_id=observation_id)
                            counters['observations'] += 1
                        else:
                            record['rejected'] = 'insufficient_depth_or_embedding'
                        envelope['detections'].append(record)
                        cv2.rectangle(annotated,(int(x1),int(y1)),(int(x2),int(y2)),(0,255,0),1)
                        cv2.putText(annotated,f'{index}:{det.class_name} {det.confidence:.2f}',
                                    (int(x1),max(12,int(y1))),cv2.FONT_HERSHEY_SIMPLEX,.35,(0,255,0),1)
                    cv2.imwrite(str(args.output/'frames'/f'{keyframe_id:05d}.jpg'),frame)
                    cv2.imwrite(str(args.output/'frames'/f'{keyframe_id:05d}_detections.jpg'),annotated)
                    np.save(args.output/'frames'/f'{keyframe_id:05d}_depth.npy',depth_m)
                    log.write(json.dumps(envelope)+'\n'); log.flush(); route.flush()
                    publisher.put(json.dumps(envelope).encode())
                    print(f'KF {keyframe_id} pose={tuple(round(v,2) for v in pose)} detections={[d.class_name for d in detections]}',flush=True)
                if time.monotonic()-last_status > 15:
                    print(dict(counters),flush=True)
                    last_status = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        metadata.update(counters=dict(counters),keyframes=keyframe_id,elapsed_s=time.monotonic()-started)
        (args.output/'run.json').write_text(json.dumps(metadata,indent=2))
        for sub in subscribers: sub.undeclare()
        publisher.undeclare(); session.close(); conn.close()

if __name__ == '__main__':
    main()
