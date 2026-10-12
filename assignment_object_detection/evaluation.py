"""Instance metrics from reviewed images, never from guessed model identities."""
from collections import Counter,defaultdict
import base64
import json
import numpy as np
from .deduplication import merge_detection_into_objects, ObjectDeduplicator

def load_keyframes(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]

def validate_annotations(frames, annotations, truth):
    ids={r['instance_id']:r['class_name'] for r in truth if r['class_name']}
    by_frame={a['keyframe_id']:a for a in annotations}
    if len(by_frame)!=len(annotations): raise ValueError('Duplicate annotation frame ID')
    for frame in frames:
        fid=frame['keyframe_id']
        if fid not in by_frame: raise ValueError(f'Frame {fid} needs visibility and detection annotations')
        a=by_frame[fid]
        if a.get('reviewed') is not True: raise ValueError(f'Frame {fid} is not reviewed')
        for identity in a.get('visible',[]):
            if identity not in ids: raise ValueError(f'Unknown visible instance {identity}')
        for det in frame['detections']:
            key=str(det['detection_index'])
            if key not in a['matches']: raise ValueError(f'Missing detection annotation {fid}:{key}')
            identity=a['matches'][key]
            if identity is not None:
                visible = det['class'] in a.get('visible_classes',[]) or identity in a.get('visible',[])
                if not visible or ids.get(identity)!=det['class']:
                    raise ValueError(f'Invalid identity/class/visibility for {fid}:{key}')
    return by_frame

def metrics(frames, annotations, truth, match_distance_m=1.0):
    reviewed=validate_annotations(frames,annotations,truth)
    gt=defaultdict(set)
    for row in truth:
        if row['class_name']: gt[row['class_name']].add(row['instance_id'])
    objects=defaultdict(lambda:defaultdict(set))
    positions=defaultdict(list)
    truth_xy={r['instance_id']:np.array([float(r['map_x']),float(r['map_y'])])
              for r in truth if r['class_name']}
    relevant=Counter(); detected=Counter()
    for frame in frames:
        annotation=reviewed[frame['keyframe_id']]
        visible_classes=set(annotation.get('visible_classes',[])) | {
            cls for cls,ids in gt.items() if ids.intersection(annotation.get('visible',[]))}
        relevant.update(visible_classes)
        correct_classes=set()
        for det in frame['detections']:
            identity=annotation['matches'][str(det['detection_index'])]
            if identity is not None: correct_classes.add(det['class'])
            if 'object_id' in det:
                evidence=objects[det['class']][det['object_id']]
                positions[(det['class'],det['object_id'])].append(det['position'][:2])
                if identity is not None: evidence.add(identity)
        detected.update(correct_classes & visible_classes)
    rows=[]
    for cls in sorted(set(gt)|set(objects)):
        records=objects[cls]; true=gt[cls]; stored=len(records)
        # One-to-one matching: two stored records of one identity count once.
        unique_correct=set()
        for oid, identities in records.items():
            if len(identities)==1:
                identity=next(iter(identities))
                fused=np.mean(positions[(cls,oid)],axis=0)
                if np.linalg.norm(fused-truth_xy[identity]) <= match_distance_m:
                    unique_correct.add(identity)
        correct=len(unique_correct)
        rows.append(dict(class_name=cls,true_instances=len(true),stored_objects=stored,correct_matches=correct,
            precision=correct/stored if stored else None,recall=correct/len(true) if true else None,
            duplicate_rate=stored/len(true) if true else None,
            detection_rate=detected[cls]/relevant[cls] if relevant[cls] else None,
            relevant_keyframes=relevant[cls],correct_detection_keyframes=detected[cls],
            false_merges=sum(len(s)>1 for s in records.values()),misses=len(true-unique_correct)))
    return rows

def replay(frames,radius,use_clip=True):
    import copy
    result=copy.deepcopy(frames)
    clusters=[]
    for frame in result:
        for det in frame['detections']:
            if 'position' not in det: continue
            vec=np.frombuffer(base64.b64decode(det['embedding']),dtype=np.float32)
            if use_clip:
                clusters,chosen,_=merge_detection_into_objects(clusters,det['class'],det['position'],vec,
                                                               frame['stamp_ns'],radius)
            else:
                candidates=ObjectDeduplicator(radius).candidate_objects(det['class'],det['position'],clusters)
                candidates.sort(key=lambda c:np.linalg.norm(c.map_position-np.array(det['position'])))
                chosen,created=ObjectDeduplicator(radius).merge_detection(det['class'],det['position'],vec,
                                                           frame['stamp_ns'],candidates[:1])
                if created: clusters.append(chosen)
            det['object_id']=next(i+1 for i,c in enumerate(clusters) if c is chosen)
    return result
