"""Download only the official aist_living_lab_1 archive linked in Stella's tutorial."""
import argparse
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile
import gdown


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('--output',default='data/aist'); args=parser.parse_args()
    root=Path(args.output); root.mkdir(parents=True,exist_ok=True)
    archive=root/'aist_living_lab_1.zip'
    if not archive.exists():
        result=gdown.download(id='1d8kADKWBptEqTF7jEVhKatBEdN7g0ikY',output=str(archive),quiet=False)
        if not result: raise RuntimeError('official Google Drive download failed; download in a browser and put the ZIP at '+str(archive))
    with ZipFile(archive) as z:
        for info in z.infolist():
            target=(root/info.filename).resolve()
            if not target.is_relative_to(root.resolve()): raise ValueError('unsafe ZIP path')
        z.extractall(root)
    videos=list(root.rglob('video.mp4'))
    if not videos: raise RuntimeError('downloaded archive did not contain the reference video')
    manifest=dict(dataset='aist_living_lab_1',source='https://stella-cv.readthedocs.io/en/latest/simple_tutorial.html',
                  drive_id='1d8kADKWBptEqTF7jEVhKatBEdN7g0ikY',archive_bytes=archive.stat().st_size,
                  sha256=hashlib.file_digest(archive.open('rb'),'sha256').hexdigest(),videos=[str(v) for v in videos])
    (root/'acquisition.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps(manifest,indent=2))


if __name__=='__main__': main()
