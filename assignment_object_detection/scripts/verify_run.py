"""Audit recorded timestamps, embeddings, count consistency, and localization."""
import argparse
import bisect
import csv
import json
from pathlib import Path
import numpy as np
from assignment_object_detection.evaluation import load_keyframes

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',required=True,type=Path);p.add_argument('--localization',required=True,type=Path)
    a=p.parse_args();frames=load_keyframes(a.run/'keyframes.jsonl')
    local=load_keyframes(a.localization);times=[r['stamp_ns'] for r in local]
    # Arrival callbacks can repeat timestamps: sort before nearest selection.
    local.sort(key=lambda r:r['stamp_ns']);times=[r['stamp_ns'] for r in local]
    with (a.run/'export/semantic_objects.csv').open() as f: objects=list(csv.DictReader(f))
    with (a.run/'export/object_observations.csv').open() as f: obs=list(csv.DictReader(f))
    by_id={int(o['object_id']):o for o in objects}
    assert sum(int(o['observation_count']) for o in objects)==len(obs)
    assert {int(o['object_id']) for o in obs} <= set(by_id)
    for obj in objects:
        assert abs(np.linalg.norm(json.loads(obj['mean_embedding']))-1)<1e-5
    for f in frames:
        assert abs(f['stamp_ns']-f['depth_stamp_ns'])<=200_000_000
        assert abs(f['stamp_ns']-f['odom_stamp_ns'])<=200_000_000
    errors=[]
    for f in frames:
        i=bisect.bisect_left(times,f['stamp_ns']);near=local[max(0,i-1):i+1]
        if not near:continue
        g=min(near,key=lambda r:abs(r['stamp_ns']-f['stamp_ns']))
        if abs(g['stamp_ns']-f['stamp_ns'])>200_000_000:continue
        errors.append(float(np.linalg.norm(np.array([f['map_x'],f['map_y']])-g['ground_truth'][:2])))
    result=dict(keyframes=len(frames),objects=len(objects),observations=len(obs),
        depth_max_delta_ms=max(abs(f['stamp_ns']-f['depth_stamp_ns']) for f in frames)/1e6,
        odom_max_delta_ms=max(abs(f['stamp_ns']-f['odom_stamp_ns']) for f in frames)/1e6,
        pose_xy_error_median_m=float(np.median(errors)),pose_xy_error_p95_m=float(np.quantile(errors,.95)),
        pose_comparisons=len(errors),all_checks_passed=True)
    (a.run/'validation.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
