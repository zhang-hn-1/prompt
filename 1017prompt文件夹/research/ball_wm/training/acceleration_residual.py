"""Versioned acceleration residual; rate-residual weights are not interchangeable."""
from __future__ import annotations
import math
import torch
from training.motion_residual import CVResidualPredictor,HISTORY,HORIZON,FRAME_DT


def integrate_acceleration(acceleration,dt):
    """Piecewise-constant acceleration, zero extra velocity/position at the anchor.

    Channels are anchor-frame x/y acceleration and planar yaw acceleration.
    For each interval: dx += dv*dt + a*dt²/2; dv += a*dt.
    """
    if not math.isfinite(dt) or dt<=0:
        raise ValueError('integration timestep must be finite and positive')
    if acceleration.ndim!=3 or acceleration.shape[-1]!=3 or acceleration.shape[1]<1 or not torch.isfinite(acceleration).all():
        raise ValueError('acceleration must be finite with shape (B,H,3)')
    velocity_change=dt*acceleration
    velocity_start=torch.cumsum(velocity_change,dim=1)-velocity_change
    return torch.cumsum(dt*velocity_start+.5*dt*dt*acceleration,dim=1)


class AccelerationResidualPredictor(CVResidualPredictor):
    """Identical trainable layers to CVResidualPredictor, different integration only."""
    parameterization='piecewise_constant_acceleration_v1'

    def forward(self,batch):
        expected={'embeddings':(HISTORY,self.embedding_dim),'proprio':(HISTORY,17),
            'past_requests':(HISTORY,10),'requests':(HORIZON,10),'prior':(HORIZON,4)}
        sizes=set()
        for key,shape in expected.items():
            value=batch[key]
            if value.ndim!=3 or tuple(value.shape[1:])!=shape or not torch.isfinite(value).all():
                raise ValueError(f'{key} must be finite with shape (B,{shape})')
            sizes.add(value.shape[0])
        if len(sizes)!=1:raise ValueError('batch dimensions must match')
        features=torch.cat([batch[k] for k in ['embeddings','proprio','past_requests']],dim=-1)
        hidden=self.context(features.flatten(1));accelerations=[]
        for step in range(HORIZON):
            hidden=self.recurrent(batch['requests'][:,step],hidden)
            accelerations.append(self.readout(hidden))
        residual=integrate_acceleration(torch.stack(accelerations,dim=1),FRAME_DT)
        prior=batch['prior'];sine,cosine=torch.sin(residual[...,2]),torch.cos(residual[...,2])
        return torch.cat([prior[...,:2]+residual[...,:2],
            (prior[...,2]*cosine+prior[...,3]*sine).unsqueeze(-1),
            (prior[...,3]*cosine-prior[...,2]*sine).unsqueeze(-1)],dim=-1)
