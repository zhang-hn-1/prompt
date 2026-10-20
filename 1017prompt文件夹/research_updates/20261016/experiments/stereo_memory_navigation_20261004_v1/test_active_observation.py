import copy,importlib
import numpy as np
import pytest
from test_guarded_response import entry,state
from feature_tracks import FeatureMemory,predict_features

def api(name):
    try:mod=importlib.import_module('active_observation')
    except ModuleNotFoundError:mod=None
    value=getattr(mod,name,None);assert callable(value),name+' missing'
    return value

def test_invalid_reference_time_rejected_before_future_payload():
    call=api('guarded_predict')
    class Trap(dict):
        def __getitem__(self,key):
            assert key=='time_s';return super().__getitem__(key)
    for t in [np.nan,np.inf,-np.inf]:
        with pytest.raises(ValueError,match='finite'):call(dict(time_s=t),[Trap(time_s=4.)],np.eye(3))
    ref=dict(time_s=2.,frame=2,eye='left',xy=np.empty((0,2)),desc=np.empty((0,32),np.uint8),pose=np.eye(4))
    assert call(ref,[Trap(time_s=4.)],np.eye(3))==predict_features(ref,[],np.eye(3))

def test_invalid_provenance_cannot_mutate_memory():
    memory=api('SafeFeatureMemory')(10)
    good=dict(world=[0,0,2],sigma_m=.01,query=[0,'left',0,0.],source=[0,'right',0,0.]);memory.update([good],0.,0.)
    for field in ['query','source']:
        bad=copy.deepcopy(good);bad[field][3]=np.nan
        with pytest.raises(ValueError,match='finite'):memory.update([bad],1.,0.)
        assert memory.snapshot(0.)==[dict(good,last_seen_s=0.)]
    for t in [np.nan,np.inf]:
        with pytest.raises(ValueError,match='finite'):memory.snapshot(t)

def test_full_matching_stop_sweep_and_unknown_are_rejected():
    gate=api('admit_observation');e=entry();e['xy']=np.array([[0,0],[.2,0],[1.5,0]])
    grid=np.zeros((101,101),np.int8)
    ok=gate(state(),e['execution'],e,grid,[-5,-5],.1)
    assert ok['admitted'] and not ok['full_3d_certified']
    grid[65,50]=1 # matching stop endpoint, no hazard at active .2m endpoint
    assert not gate(state(),e['execution'],e,grid,[-5,-5],.1)['admitted']
    grid[:]=-1;assert not gate(state(),e['execution'],e,grid,[-5,-5],.1)['admitted']

def test_admission_uses_actual_joints_and_current_yaw():
    gate=api('admit_observation');e=entry();grid=np.zeros((101,101),np.int8)
    s=state(np.pi/2);r=gate(s,e['execution'],e,grid,[-5,-5],.1)
    np.testing.assert_allclose(r['world_xy'],[[0,0],[.15,.1],[.27,.6],[.1,.7]],atol=1e-10)
    s['joint_velocity'][1]=.2
    assert not gate(s,e['execution'],e,grid,[-5,-5],.1)['admitted']
