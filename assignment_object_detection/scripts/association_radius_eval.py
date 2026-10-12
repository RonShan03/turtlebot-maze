#!/usr/bin/env python3
"""Evaluate candidate association radii for deduplication."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from assignment_object_detection.evaluation import load_keyframes,metrics,replay


def main() -> None:
    parser = argparse.ArgumentParser(description="Association-radius sweep")
    parser.add_argument("--radii", nargs="+", type=float, default=[0.25, 0.5, 0.75, 1.0])
    parser.add_argument("--output", default="assignment_object_detection/output/radius_eval.csv")
    parser.add_argument('--run',required=True,type=Path)
    parser.add_argument('--truth',required=True,type=Path)
    parser.add_argument('--annotations',required=True,type=Path)
    args = parser.parse_args()

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    frames=load_keyframes(args.run/'keyframes.jsonl')
    annotations=json.loads(args.annotations.read_text())
    with args.truth.open() as f: truth=list(csv.DictReader(f))
    rows=[]
    for radius in args.radii:
        for clip in (True,False):
            for row in metrics(replay(frames,radius,clip),annotations,truth):
                rows.append(dict(radius_m=radius,tie_break='clip' if clip else 'nearest_position',**row))
    with out.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    print(f'Wrote measured replay results to {out}')


if __name__ == "__main__":
    main()
