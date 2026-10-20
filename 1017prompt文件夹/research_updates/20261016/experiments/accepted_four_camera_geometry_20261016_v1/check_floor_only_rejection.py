"""A visible floor alone must never create a witnessed obstacle-floor interface."""
import json,numpy as np
from contact_oracle import select_labels
from qualify import INPUT,AX

a=np.load(INPUT/'recovery_run/frames/frame_000.npz')
T=a['front_left_camera_usd_world_label'];R=T[:3,:3]@AX
s=json.loads((INPUT/'recovery_run/capture_seal.json').read_text());F=s['initial_local_identity'][0]['optics'][0]/18*640
y,x=np.indices((480,640));rays=np.stack([(x-319.5)/F,(y-239.5)/F,np.ones_like(x)],axis=-1)
direction=rays@R.T
depth=np.divide(-T[2,3],direction[...,2],out=np.full((480,640),np.inf),where=abs(direction[...,2])>1e-12)
depth[(depth<.01)|(depth>1000)]=np.inf
theta=np.arange(1440)*2*np.pi/1440
objects=[dict(center=[0.,0.,.5],radius=.35,height=1.)]
points=np.c_[.35*np.cos(theta),.35*np.sin(theta),np.zeros(1440)]
labels,_,_=select_labels(points,np.zeros(1440,int),objects,T,depth,np.zeros((480,640),bool),F)
assert len(labels)==0,f'Floor-only rendering fabricated {len(labels)} obstacle-contact labels'
print('PASS: floor-only rendering produces zero contact labels')
