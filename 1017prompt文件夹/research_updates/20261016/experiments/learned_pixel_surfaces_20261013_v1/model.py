"""New independent pixel-category and range objective. No old BEV checkpoint."""
import torch
from torch import nn
from torch.nn import functional as F
class Block(nn.Sequential):
 def __init__(self,a,b):
  super().__init__(nn.Conv2d(a,b,3,padding=1),nn.GroupNorm(4,b),nn.SiLU(),nn.Conv2d(b,b,3,padding=1),nn.GroupNorm(4,b),nn.SiLU())
class PixelSurface(nn.Module):
 def __init__(self):
  super().__init__();self.a=Block(10,16);self.b=Block(16,32);self.c=Block(32,64);self.d=Block(96,32);self.e=Block(48,16);self.head=nn.Conv2d(16,8,1)
 def forward(self,x):
  a=self.a(x);b=self.b(F.avg_pool2d(a,2));c=self.c(F.avg_pool2d(b,2))
  d=self.d(torch.cat([F.interpolate(c,size=b.shape[-2:],mode='bilinear',align_corners=False),b],1))
  z=self.head(self.e(torch.cat([F.interpolate(d,size=a.shape[-2:],mode='bilinear',align_corners=False),a],1)))
  return z[:,:6].reshape(-1,2,3,*z.shape[-2:]),.5+2.5*z[:,6:].sigmoid()
