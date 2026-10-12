#!/usr/bin/env python3
"""Generate a simple semantic map plot from object detections."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot semantic map estimates")
    parser.add_argument('--input',required=True,type=Path,help='semantic_objects.csv')
    parser.add_argument('--route',required=True,type=Path,help='route.jsonl from capture')
    parser.add_argument('--map',type=Path,default=Path('tb_worlds/maps/house_world_map.yaml'))
    parser.add_argument('--output',required=True,type=Path)
    args = parser.parse_args()

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import yaml
    from PIL import Image
    import numpy as np
    meta=yaml.safe_load(args.map.read_text())
    background=np.asarray(Image.open(args.map.parent/meta['image']))
    resolution=meta['resolution']; ox,oy,_=meta['origin']; h,w=background.shape
    fig,ax=plt.subplots(figsize=(13,10))
    ax.imshow(background,cmap='gray',vmin=0,vmax=255,extent=[ox,ox+w*resolution,oy,oy+h*resolution])
    route=[json.loads(line) for line in args.route.read_text().splitlines() if line.strip()]
    if route:
        ax.plot([p['x'] for p in route],[p['y'] for p in route],color='tab:blue',lw=1,alpha=.6,label='Accepted synchronized poses')
        ax.scatter([route[0]['x']],[route[0]['y']],c='lime',edgecolors='black',marker='s',s=65,label='Start')
    with args.input.open(newline='') as f: objects=list(csv.DictReader(f))
    for cls in sorted({r['class_name'] for r in objects}):
        rows=[r for r in objects if r['class_name']==cls]
        ax.scatter([float(r['map_x']) for r in rows],[float(r['map_y']) for r in rows],s=36,label=cls)
        for r in rows:
            ax.annotate(f"{cls} #{r['object_id']}",(float(r['map_x']),float(r['map_y'])),
                        xytext=(4,4),textcoords='offset points',fontsize=7)
    ax.set(xlim=(-10,10),ylim=(-6.5,6.5),xlabel='Map x (m)',ylabel='Map y (m)',title='House semantic objects and traversal coverage')
    ax.set_aspect('equal'); ax.legend(loc='upper left',bbox_to_anchor=(1,1),fontsize=8)
    fig.tight_layout(); fig.savefig(out,dpi=180); plt.close(fig)
    print(f'Wrote {out}: {len(objects)} objects, {len(route)} poses')


if __name__ == "__main__":
    main()
