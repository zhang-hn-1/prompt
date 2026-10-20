"""Independent research gate. Coefficients are not calibrated probabilities."""
import torch
from torch import nn

def state_features(batch):
    p=batch['proprio']
    if p.ndim!=3 or tuple(p.shape[1:])!=(5,17) or not torch.isfinite(p).all():
        raise ValueError('proprio must be finite (B,5,17)')
    # Each state is in its own body frame: the difference is a trend, not acceleration.
    return torch.cat([p[:,-1],p[:,-1]-p[:,0]],dim=-1)

def pose_delta(pose,prior):
    angle=torch.atan2(pose[...,2]*prior[...,3]-pose[...,3]*prior[...,2],
                      pose[...,3]*prior[...,3]+pose[...,2]*prior[...,2])
    return torch.cat([pose[...,:2]-prior[...,:2],angle.unsqueeze(-1)],dim=-1)

def scale_targets(prediction,prior,target):
    c=pose_delta(prediction,prior);r=pose_delta(target,prior)
    energy=c.square().sum(-1)
    coefficient=((c*r).sum(-1)/energy.clamp_min(1e-12)).clamp(0,1)
    coefficient=torch.where(energy>1e-12,coefficient,torch.zeros_like(coefficient))
    return coefficient.detach(),torch.where(energy>1e-12,energy,torch.zeros_like(energy)).detach()

def blend_pose(prior,prediction,coefficient):
    if coefficient.shape!=prior.shape[:-1] or not torch.isfinite(coefficient).all() or (coefficient<0).any() or (coefficient>1).any():
        raise ValueError('coefficient must be finite in [0,1] with shape (B,H)')
    delta=pose_delta(prediction,prior)*coefficient.unsqueeze(-1)
    sine,cosine=torch.sin(delta[...,2]),torch.cos(delta[...,2])
    out=torch.cat([prior[...,:2]+delta[...,:2],
        (prior[...,2]*cosine+prior[...,3]*sine).unsqueeze(-1),
        (prior[...,3]*cosine-prior[...,2]*sine).unsqueeze(-1)],dim=-1)
    # Preserve literal fallback and identity despite floating-point trig roundoff.
    out=torch.where((coefficient==0).unsqueeze(-1),prior,out)
    return torch.where((coefficient==1).unsqueeze(-1),prediction,out)

class StateConfidenceGate(nn.Module):
    def __init__(self,mean,scale):
        super().__init__()
        if mean.shape!=(34,) or scale.shape!=(34,) or not torch.isfinite(mean).all() or not torch.isfinite(scale).all() or (scale<=0).any():
            raise ValueError('normalization must be finite with positive scale')
        self.register_buffer('feature_mean',mean.clone());self.register_buffer('feature_scale',scale.clone())
        self.linear=nn.Linear(34,5)
        nn.init.zeros_(self.linear.weight);nn.init.zeros_(self.linear.bias)

    def coefficients(self,features,*,apply_cutoff=True):
        if features.ndim!=2 or features.shape[1]!=34 or not torch.isfinite(features).all():
            raise ValueError('features must be finite (B,34)')
        raw=torch.sigmoid(self.linear((features-self.feature_mean)/self.feature_scale))
        return torch.where(raw>=.5,raw,torch.zeros_like(raw)) if apply_cutoff else raw

    def forward(self,batch):
        return self.coefficients(state_features(batch))

def fit_gate(features,target,energy,*,steps=500):
    if features.ndim!=2 or features.shape[1]!=34 or target.shape!=(len(features),5) or energy.shape!=target.shape:
        raise ValueError('invalid fit shapes')
    if not all(torch.isfinite(x).all() for x in [features,target,energy]) or (target<0).any() or (target>1).any() or (energy<0).any():
        raise ValueError('invalid fit values')
    gate=StateConfidenceGate(features.mean(0),features.std(0,unbiased=False).clamp_min(1e-4))
    optimizer=torch.optim.Adam(gate.parameters(),lr=.01)
    weight=energy/energy.mean(0,keepdim=True).clamp_min(1e-12);history=[]
    for step in range(steps):
        optimizer.zero_grad();coeff=gate.coefficients(features,apply_cutoff=False)
        calibration=(weight*(coeff-target).square()).mean()
        regularization=.01*gate.linear.weight.square().mean();loss=calibration+regularization
        loss.backward();optimizer.step()
        history.append({'step':step+1,'loss':float(loss.detach()),'calibration_loss':float(calibration.detach()),'regularization':float(regularization.detach())})
    return gate.eval(),history

class GatedPredictor(nn.Module):
    def __init__(self,base,gate):
        super().__init__();self.base=base.eval().requires_grad_(False);self.gate=gate
    def forward(self,batch):
        return blend_pose(batch['prior'],self.base(batch),self.gate(batch))
