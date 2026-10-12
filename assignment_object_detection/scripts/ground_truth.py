"""Inventory distinct SDF model instances; explicit COCO mapping is reviewable."""
import argparse
import csv
import xml.etree.ElementTree as ET
from pathlib import Path

# These categories are semantic interpretations of the furnished house assets.
# Coffee/balcony tables and the writing desk are grouped with COCO dining table.
# The tablet is not a laptop; composite kitchen subparts are outside this inventory.
CLASS_PREFIXES={'Chair':'chair','Sofa':'couch','Bed_':'bed','TV_':'tv',
    'KitchenTable':'dining table','CoffeeTable':'dining table','BalconyTable':'dining table',
    'ReadingDesk':'dining table',
    'Refrigerator':'refrigerator','Vase':'vase','Ball_':'sports ball'}

def inventory(world):
    rows=[]
    for model in ET.parse(world).findall('./world/model'):
        uri=model.findtext('include/uri')
        if not uri: continue
        name=model.attrib['name']
        pose=[float(v) for v in model.findtext('pose','0 0 0 0 0 0').split()]
        cls=next((value for prefix,value in CLASS_PREFIXES.items() if name.startswith(prefix)),'')
        rows.append(dict(instance_id=name,class_name=cls,map_x=pose[0],map_y=pose[1],map_z=pose[2],
                         roll=pose[3],pitch=pose[4],yaw=pose[5],asset=uri))
    return rows

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--world',type=Path,default=Path('tb_worlds/worlds/house_world.sdf.xacro'))
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(); rows=inventory(a.world)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    from collections import Counter
    print(dict(Counter(r['class_name'] for r in rows if r['class_name'])))

if __name__=='__main__': main()
