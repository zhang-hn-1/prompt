"""Current RGB/IMU -> local visible-surface probabilities, no GT/SGBM input."""
import torch
from torch import nn
from torch.nn import functional as F
class VisibleMapNet(nn.Module):
 def __init__(self):
  super().__init__();layers=[];inc=6
  for out in [16,32,48,64]:layers += [nn.Conv2d(inc,out,3,stride=2,padding=1),nn.ReLU()];inc=out
  self.encoder=nn.Sequential(*layers);self.fc=nn.Sequential(nn.Linear(64*8*10+3,256),nn.ReLU(),nn.Linear(256,32*5*8),nn.ReLU());self.decoder=nn.Sequential(nn.Conv2d(32,32,3,padding=1),nn.ReLU(),nn.Conv2d(32,5,1))
 def forward(self,rgb,gravity):
  if rgb.ndim!=4 or rgb.shape[1:]!=(6,120,160) or gravity.shape!=(rgb.shape[0],3):raise ValueError('current input shape')
  h=self.encoder(rgb).flatten(1);h=self.fc(torch.cat([h,gravity],1)).reshape(-1,32,5,8);return self.decoder(F.interpolate(h,size=(40,60),mode='bilinear',align_corners=False))
def tentative(logits):
 p=logits.softmax(1);confidence,label=p.max(1);label=torch.where(confidence>=.5,label,torch.zeros_like(label));return label, p
