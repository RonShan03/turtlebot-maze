"""Plot actual saved TUM trajectories and MessagePack landmarks; no fabricated output."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    p=argparse.ArgumentParser(); p.add_argument('--results',required=True); args=p.parse_args()
    folder=Path(args.results)
    rows=np.loadtxt(folder/'frame_trajectory.txt',ndmin=2)
    if rows.shape[1]!=8 or not len(rows): raise ValueError('expected nonempty TUM timestamp xyz quaternion trajectory')
    xyz=rows[:,1:4]
    report=dict(trajectory_samples=len(rows),timestamp_duration_s=float(rows[-1,0]-rows[0,0]),
                path_length_arbitrary_units=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum()),
                note='Monocular AIST map scale is arbitrary; no reference ground truth error reported')
    landmarks=None
    if (folder/'map.msg').exists():
        import msgpack
        with (folder/'map.msg').open('rb') as f: data=msgpack.unpack(f,raw=False,strict_map_key=False)
        if isinstance(data,dict):
            report['map_sections']=list(data.keys())
            entries=data.get('landmarks',{})
            if isinstance(entries,dict): entries=entries.values()
            points=[entry['pos_w'] for entry in entries if isinstance(entry,dict) and 'pos_w' in entry]
            if points: landmarks=np.array(points,float)
            report['landmarks']=len(points)
            report['keyframes']=len(data.get('keyframes',{}))
    fig=plt.figure(figsize=(9,6)); ax=fig.add_subplot(projection='3d')
    if landmarks is not None:
        ax.scatter(*landmarks[::max(1,len(landmarks)//15000)].T,s=.4,alpha=.2,label='saved sparse landmarks')
    ax.plot(*xyz.T,c='red',label='saved frame trajectory')
    ax.set(xlabel='X (arbitrary units)',ylabel='Y',zlabel='Z',title='Actual StellaVSLAM AIST reconstruction')
    ax.legend(); fig.tight_layout(); fig.savefig(folder/'reconstruction.png',dpi=160); plt.close(fig)
    (folder/'summary.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
