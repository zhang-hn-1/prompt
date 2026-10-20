import numpy as np
from information_policy import InformationPolicy
from local_map import BeliefGrid
from test_policy import P,observation

def test_surface_branch_adds_only_supported_floor_cells():
    mesh=np.array([[[1.,-.1,0],[1.1,-.1,0]],[[1.,.1,0],[1.1,.1,0]]])
    policies=[]
    for mode in ('points','surface'):
        p=InformationPolicy(P,'visible',mode)
        packet={'time_s':0.,'root_xyz':np.array([0.,0.,.4]),'points_world':mesh.reshape(-1,3),'ground_meshes':[mesh]}
        p.set_current_packet(packet);action=p.act(observation(),np.array([2.,0.]));policies.append(p)
        assert np.all(action==0)
    a,b=policies
    assert a.belief.labels[210,200]==-1 and b.belief.labels[210,200]==0
    assert b.belief.labels[205,200]==-1
    assert b.records[0]['known_free']>a.records[0]['known_free']

def test_continuous_observed_floor_has_route_and_driving_request():
    x,y=np.meshgrid(np.arange(-1,3.01,.1),np.arange(-1,1.01,.1),indexing='ij');mesh=np.stack((x,y,np.zeros_like(x)),axis=-1)
    p=InformationPolicy(P,'visible','surface');p.set_current_packet({'time_s':0.,'points_world':mesh.reshape(-1,3),'ground_meshes':[mesh],'root_xyz':np.array([0.,0.,.4])})
    action=p.act(observation(),np.array([2.,0.]))
    assert p.records[-1]['route_reason']=='goal' and action[0]>0
