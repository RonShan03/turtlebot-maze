"""Compute per-class evaluation after reviewing captured keyframes."""
import argparse
import csv
import json
from pathlib import Path
from assignment_object_detection.evaluation import load_keyframes,metrics

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',required=True,type=Path)
    p.add_argument('--truth',required=True,type=Path)
    p.add_argument('--annotations',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    a=p.parse_args()
    frames=load_keyframes(a.run/'keyframes.jsonl')
    with a.truth.open() as f: truth=list(csv.DictReader(f))
    rows=metrics(frames,json.loads(a.annotations.read_text()),truth)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    print(json.dumps(rows,indent=2))

if __name__=='__main__': main()
