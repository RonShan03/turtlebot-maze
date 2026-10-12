"""Measure same-instance localization scatter from manually reviewed observations."""
import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path
import numpy as np
from assignment_object_detection.evaluation import load_keyframes,validate_annotations
from assignment_object_detection.geometry import _internal_bbox_region

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',required=True,type=Path);p.add_argument('--truth',required=True,type=Path)
    p.add_argument('--annotations',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    a=p.parse_args(); frames=load_keyframes(a.run/'keyframes.jsonl')
    with a.truth.open() as f: truth=list(csv.DictReader(f))
    labels=validate_annotations(frames,json.loads(a.annotations.read_text()),truth)
    points=defaultdict(list); depths=defaultdict(list); boxes=defaultdict(list); poses=defaultdict(list)
    roi_mad=defaultdict(list)
    archive=np.load(a.run/'depth_frames.npz') if (a.run/'depth_frames.npz').exists() else None
    for frame in frames:
        for det in frame['detections']:
            identity=labels[frame['keyframe_id']]['matches'][str(det['detection_index'])]
            if identity and 'position' in det:
                points[identity].append(det['position']);depths[identity].append(det['median_depth_m'])
                boxes[identity].append(det['bbox']);poses[identity].append([frame['map_x'],frame['map_y'],frame['map_yaw']])
                key=f"{frame['keyframe_id']:05d}_depth"
                depth=archive[key] if archive is not None else np.load(a.run/'frames'/f'{key}.npy')
                x1,y1,x2,y2=_internal_bbox_region(det['bbox'])
                roi=depth[y1:y2,x1:x2]; valid=roi[np.isfinite(roi)&(roi>0)]
                roi_mad[identity].append(float(np.median(np.abs(valid-np.median(valid)))))
    rows=[]
    for identity,values in points.items():
        values=np.array(values);n=len(values); centered=np.linalg.norm(values-values.mean(axis=0),axis=1)
        pairwise=np.linalg.norm(values[:,None,:]-values[None,:,:],axis=2)[np.triu_indices(n,1)]
        bbox=np.array(boxes[identity]);pose=np.array(poses[identity])
        rows.append(dict(instance_id=identity,n=n,position_scatter_p95_m=float(np.quantile(centered,.95)),
            pair_distance_p95_m=float(np.quantile(pairwise,.95)) if n>1 else None,
            median_depth_std_m=float(np.std(depths[identity])),bbox_width_std_px=float(np.std(bbox[:,2]-bbox[:,0])),
            bbox_height_std_px=float(np.std(bbox[:,3]-bbox[:,1])),
            central_roi_depth_mad_median_m=float(np.median(roi_mad[identity])),
            robot_xy_span_m=float(np.linalg.norm(np.ptp(pose[:,:2],axis=0)))))
    if archive is not None: archive.close()
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('w',newline='') as f:
        if rows:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    print(json.dumps(rows,indent=2))

if __name__=='__main__':main()
