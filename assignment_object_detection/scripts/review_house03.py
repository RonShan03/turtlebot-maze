"""Run-specific, AI-assisted visual review of saved RGB contact sheets.

This transcription is evidence for house03 only, not an automatic annotator.
Projected SDF centers and adjacent views assist identity resolution; ambiguous
multi-object boxes remain null. Partial physical objects count as visible.
"""
import json
from pathlib import Path

RUN = Path('assignment_object_detection/runs/house03')
ALIASES = {'B':'Bed_01_001','C':'SofaC_01_001','R':'Refrigerator_01_001','E':'ReadingDesk_01_001',
           'T':'KitchenTable_01_001','N':'BalconyTable_01_001','F':'CoffeeTable_01_001',
           'S1':'Ball_01_001','S2':'Ball_01_003','TV1':'TV_01_001','TV2':'TV_02_001',
           **{f'A{i}':f'ChairA_01_00{i}' for i in range(1,6)},
           **{f'D{i}':f'ChairD_01_00{i}' for i in range(1,4)}}
CODES = {'t':'tv','c':'chair','d':'dining table','b':'bed','s':'sports ball',
         'r':'refrigerator','o':'couch','v':'vase'}
VISIBILITY = [
 (1,2,'cdr'),(3,3,'odr'),(4,6,'od'),(7,8,'ocs'),(9,9,'cs'),
 (10,11,'bcs'),(12,14,'cs'),(15,15,'ts'),(16,20,'t'),(21,24,'to'),
 (25,30,'t'),(31,32,'tcr'),(33,33,'r'),(34,38,'t'),(39,39,'tr'),
 (40,45,'cdr'),(46,47,'cdrs'),(48,50,'cds'),(51,51,'ocds'),
 (52,54,'od'),(55,55,'ocd'),(56,60,'cds'),(61,62,'s'),
 (63,63,'cds'),(64,64,'cd'),(65,74,'cdr'),(75,76,'r'),(77,80,''),
 (81,84,'r'),(85,87,'cr'),(88,96,''),(97,97,'c'),(98,100,'cdo'),
 (101,103,'cdr'),(104,106,'r'),(107,108,'t'),(109,110,'tod'),
 (111,113,'od'),(114,117,'ocd'),(118,119,'cd'),(120,121,'cds'),
 (122,133,'cd'),(134,136,''),(137,139,'c'),(140,142,'cd'),(143,143,'c'),
 (144,144,''),(145,145,'t'),(146,146,''),(147,148,'t'),(149,149,'ts'),
 (150,150,'s'),(151,152,'ts'),(153,153,'tb'),(154,157,'tbcd'),
 (158,158,'cd'),(159,159,''),(160,160,'d'),(161,163,'bcd'),(164,164,'b'),
 (165,166,'bcd'),(167,168,'bcd'),(169,170,''),(171,171,'d'),(172,173,'cd'),
 (174,175,'bcd'),(176,176,'cd'),(177,177,'d'),(178,182,''),(183,184,'c'),(185,186,'cs'),
 (187,187,'s'),(188,190,'ocd'),(191,193,''),(194,195,'s'),(196,197,'cs'),
 (198,199,'c'),(200,200,'cs'),(201,202,'c'),(203,207,''),(208,209,'o'),
 (210,214,'ocd'),(215,219,'ocds'),(220,223,'cds'),(224,225,'cdr')
]
MATCHES = {
 1:{0:'A2',1:'T',2:'A3',4:'A3'},2:{0:'A2'},7:{0:'S2'},11:{1:'S2'},
 12:{0:'S2'},14:{0:'S2'},31:{0:'R'},32:{0:'A1',1:'R',3:'A2'},40:{1:'R'},
 43:{0:'A1',1:'A2',2:'R'},44:{0:'A1'},
 45:{0:'A4',1:'A1',2:'A2',3:'A1',4:'R'},
 46:{1:'A2',2:'A1',3:'A1',4:'A3',5:'A3'},
 47:{1:'A1',2:'A2',3:'A1',4:'A2'},48:{1:'T',2:'A4'},57:{0:'D1'},
 63:{1:'A3',2:'A4'},64:{0:'A4',1:'A3'},
 65:{0:'A2',1:'A3',2:'A3',3:'R'},66:{1:'A2',2:'A2'},
 67:{0:'A2',1:'R'},68:{0:'A1'},69:{0:'R'},74:{1:'R'},151:{1:'S2'},
 161:{1:'E',2:'A5'},162:{0:'E'},163:{0:'E'},165:{0:'A5'},166:{1:'E'},
 167:{1:'A5'},172:{1:'E'},174:{0:'A5',1:'B'},175:{1:'A5',2:'E'},185:{0:'S2'},
 186:{1:'S2'},195:{0:'S2'},215:{1:'S1'},221:{0:'T'},223:{0:'A2'},225:{1:'A2',2:'T'}
}

def main():
    frames=[json.loads(s) for s in (RUN/'keyframes.jsonl').read_text().splitlines()]
    visibility={i:[CODES[c] for c in code] for lo,hi,code in VISIBILITY for i in range(lo,hi+1)}
    annotations=[]
    for frame in frames:
        fid=frame['keyframe_id']
        if fid not in visibility: continue
        matches={str(d['detection_index']):None for d in frame['detections']}
        for index,alias in MATCHES.get(fid,{}).items():
            if str(index) not in matches: raise ValueError(f'Unknown detection {fid}:{index}')
            matches[str(index)]=ALIASES[alias]
        annotations.append(dict(keyframe_id=fid,reviewed=True,visible_classes=visibility[fid],matches=matches))
    (RUN/'annotations.json').write_text(json.dumps(annotations,indent=2))
    print(f'Transcribed {len(annotations)} reviewed keyframes of {len(frames)}')

if __name__=='__main__': main()
