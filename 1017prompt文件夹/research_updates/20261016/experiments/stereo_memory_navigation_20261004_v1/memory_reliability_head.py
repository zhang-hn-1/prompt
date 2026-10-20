"""One low-order learned static-memory reliability head; no context/file inputs."""
import numpy as np
import torch

class ReliabilityHead(torch.nn.Module):
 def __init__(self,mean,scale):
  super().__init__();mean=np.asarray(mean,float);scale=np.asarray(scale,float)
  if mean.shape!=(21,) or scale.shape!=(21,) or not np.isfinite(mean).all() or not np.isfinite(scale).all() or (scale<=0).any():raise ValueError('fit-only finite standardization required')
  self.register_buffer('mean',torch.tensor(mean,dtype=torch.float32));self.register_buffer('scale',torch.tensor(scale,dtype=torch.float32));self.network=torch.nn.Sequential(torch.nn.Linear(21,16),torch.nn.Tanh(),torch.nn.Linear(16,1))
 def forward(self,features):
  return self.network((features-self.mean)/self.scale).squeeze(-1)
 def predict(self,features):
  if not isinstance(features,np.ndarray) or features.ndim!=2 or features.shape[1]!=21 or not np.isfinite(features).all():raise ValueError('only finite Nx21 current/past feature matrix accepted')
  self.eval()
  with torch.no_grad():return torch.sigmoid(self(torch.tensor(features,dtype=torch.float32))).numpy()
