"""One frozen shared per-camera RGB/calibration/gravity contact classifier.

Independent visible-contact target. No distance head or rejected model weights.
Weights are not fitted until complete data/geometry qualification passes.
"""
import torch
from torch import nn
import torch.nn.functional as F

class ContactNet(nn.Module):
    def __init__(self):
        super().__init__()
        def block(a,b):return nn.Sequential(nn.Conv2d(a,b,3,padding=1),nn.ReLU(),nn.Conv2d(b,b,3,padding=1),nn.ReLU())
        self.enc1=block(9,16);self.enc2=block(16,32);self.center=block(32,64)
        self.dec2=block(96,32);self.dec1=block(48,16);self.output=nn.Conv2d(16,1,1)
    def forward(self,x):
        a=self.enc1(x);b=self.enc2(F.avg_pool2d(a,2));c=self.center(F.avg_pool2d(b,2))
        d=self.dec2(torch.cat([F.interpolate(c,size=b.shape[-2:],mode='bilinear',align_corners=False),b],1))
        e=self.dec1(torch.cat([F.interpolate(d,size=a.shape[-2:],mode='bilinear',align_corners=False),a],1))
        return self.output(e)
