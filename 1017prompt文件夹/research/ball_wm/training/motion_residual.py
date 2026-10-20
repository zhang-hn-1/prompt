"""A standalone causal motion residual; it does not replace the navigation model."""
from __future__ import annotations
import numpy as np
import torch
from torch import nn
from training.ball_wm_dataset import _proprio17
from training.pose_utils import planar_yaw_from_wxyz, relative_planar_pose
from scripts.evaluate_wm_diverse_prediction import motion_baselines

HISTORY, HORIZON, FRAME_DT, BLOCK = 5, 5, .1, 5


def build_window(states, embeddings, start):
    """Future state is used only for labels, never prior or observed features."""
    frames=np.arange(start,start+HISTORY)
    observed=BLOCK*(frames+1)-1
    anchor=int(observed[-1]); targets=anchor+BLOCK*np.arange(1,HORIZON+1)
    times=np.asarray(states['state_timestamp'])
    observed_dt=float(times[anchor]-times[observed[-2]])
    if not np.isclose(observed_dt,FRAME_DT,atol=1e-6,rtol=0):
        raise ValueError('observed frames must be 10 Hz')
    if not np.allclose(times[targets]-times[anchor],np.arange(1,6)*FRAME_DT,atol=1e-6,rtol=0):
        raise ValueError('label timestamps must match the predefined horizons')
    yaw=planar_yaw_from_wxyz(states['quat'][anchor])
    previous=planar_yaw_from_wxyz(states['quat'][observed[-2]])
    _,prior=motion_baselines(states['lin_vel'][anchor],yaw,previous,observed_dt,np.arange(1,6)*FRAME_DT)
    requests=np.asarray(states['commanded_action'][anchor+1:anchor+1+BLOCK*HORIZON]).reshape(HORIZON,10)
    # Past requests end at the observed anchor, unlike future command blocks.
    past=np.stack([states['commanded_action'][i-BLOCK+1:i+1].reshape(10) for i in observed])
    target=np.stack([relative_planar_pose(states['pos'][anchor],states['quat'][anchor],states['pos'][j],states['quat'][j]) for j in targets])
    values=dict(embeddings=np.asarray(embeddings)[frames].copy(),
        proprio=np.stack([_proprio17(states,int(i),velocity_frame='base') for i in observed]),
        past_requests=past,requests=requests.copy(),prior=prior,target=target)
    return {k:torch.as_tensor(np.asarray(v,dtype=np.float32).copy()) for k,v in values.items()}


class CVResidualPredictor(nn.Module):
    """Encode observed history, then integrate command-conditioned residual rates."""
    def __init__(self, embedding_dim=192, hidden_dim=64):
        super().__init__()
        self.embedding_dim=embedding_dim
        self.context=nn.Sequential(nn.Linear(HISTORY*(embedding_dim+17+10),hidden_dim),nn.Tanh())
        self.recurrent=nn.GRUCell(10,hidden_dim)
        self.readout=nn.Linear(hidden_dim,3)
        nn.init.zeros_(self.readout.weight); nn.init.zeros_(self.readout.bias)

    def forward(self,batch):
        expected={'embeddings':(HISTORY,self.embedding_dim),'proprio':(HISTORY,17),
            'past_requests':(HISTORY,10),'requests':(HORIZON,10),'prior':(HORIZON,4)}
        sizes=set()
        for key,shape in expected.items():
            value=batch[key]
            if value.ndim!=3 or tuple(value.shape[1:])!=shape or not torch.isfinite(value).all():
                raise ValueError(f'{key} must be finite with shape (B,{shape})')
            sizes.add(value.shape[0])
        if len(sizes)!=1: raise ValueError('batch dimensions must match')
        features=torch.cat([batch[k] for k in ['embeddings','proprio','past_requests']],dim=-1)
        hidden=self.context(features.flatten(1)); accumulated=hidden.new_zeros((hidden.shape[0],3)); result=[]
        for step in range(HORIZON):
            hidden=self.recurrent(batch['requests'][:,step],hidden)
            accumulated=accumulated+FRAME_DT*self.readout(hidden)
            prior=batch['prior'][:,step]
            sine,cosine=torch.sin(accumulated[:,2]),torch.cos(accumulated[:,2])
            result.append(torch.cat([prior[:,:2]+accumulated[:,:2],
                (prior[:,2]*cosine+prior[:,3]*sine)[:,None],
                (prior[:,3]*cosine-prior[:,2]*sine)[:,None]],dim=-1))
        return torch.stack(result,dim=1)


def motion_loss(predicted,target):
    """Equal-weight XY squared norm (m²) plus wrapped yaw squared (rad²), steps 1/3/5."""
    p,t=predicted[:,[0,2,4]],target[:,[0,2,4]]
    angle=torch.atan2(p[...,2]*t[...,3]-p[...,3]*t[...,2],p[...,2]*t[...,2]+p[...,3]*t[...,3])
    return ((p[...,:2]-t[...,:2]).square().sum(-1)+angle.square()).mean()
