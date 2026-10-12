import pytest
from assignment_object_detection.evaluation import metrics

def test_duplicate_merge_miss_and_visibility_denominator():
    truth=[dict(instance_id=x,class_name='chair',map_x=i,map_y=0) for i,x in enumerate(('a','b','c'))]
    frames=[dict(keyframe_id=1,detections=[
        dict(detection_index=0,**{'class':'chair'},object_id=1),
        dict(detection_index=1,**{'class':'chair'},object_id=2),
        dict(detection_index=2,**{'class':'chair'},object_id=3)]),
        dict(keyframe_id=2,detections=[dict(detection_index=0,**{'class':'chair'},object_id=3)]),
        dict(keyframe_id=3,detections=[])]
    labels=[dict(keyframe_id=1,reviewed=True,visible=['a','b'],matches={'0':'a','1':'a','2':'a'}),
            dict(keyframe_id=2,reviewed=True,visible=['b'],matches={'0':'b'}),
            dict(keyframe_id=3,reviewed=True,visible=['c'],matches={})]
    for frame in frames:
        for det in frame['detections']: det['position']=[0,0,0]
    row=metrics(frames,labels,truth)[0]
    assert row['correct_matches']==1
    assert row['precision']==pytest.approx(1/3)
    assert row['recall']==pytest.approx(1/3)
    assert row['false_merges']==1 and row['misses']==2
    assert row['detection_rate']==pytest.approx(2/3)
    assert row['duplicate_rate']==1

def test_unreviewed_data_cannot_produce_metrics():
    with pytest.raises(ValueError,match='not reviewed'):
        metrics([dict(keyframe_id=1,detections=[])],[dict(keyframe_id=1,reviewed=False)],[])
