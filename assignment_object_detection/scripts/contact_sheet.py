"""Create labeled contact sheets for reviewing physical identities and visibility."""
import argparse
from pathlib import Path
from PIL import Image,ImageDraw

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',required=True,type=Path)
    a=p.parse_args(); files=sorted((a.run/'frames').glob('*_detections.jpg'))
    for start in range(0,len(files),24):
        chunk=files[start:start+24]
        sheet=Image.new('RGB',(320*4,265*6),'white');draw=ImageDraw.Draw(sheet)
        for i,path in enumerate(chunk):
            x=(i%4)*320;y=(i//4)*265
            sheet.paste(Image.open(path),(x,y+25));draw.text((x+4,y+4),f'Keyframe {path.stem.split("_")[0]}',fill='black')
        sheet.save(a.run/f'contact_{start//24+1:02d}.jpg')

if __name__=='__main__': main()
