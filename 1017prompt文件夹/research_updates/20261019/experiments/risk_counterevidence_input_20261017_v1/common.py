from pathlib import Path
import hashlib,json,sys
import numpy as np
D=Path(__file__).resolve().parent; W=D.parents[1]
BASE=W/'experiments/four_camera_weak_baseline_20261017_v1'; RUN=BASE/'e1_run'
NAMES=('front_left','front_right','rear_left','rear_right')
PAIRS=((0,1),(3,2)); F=294.77087816197377; AX=np.diag([1.,-1.,-1.])
sys.path.insert(0,str(W/'experiments/user_fov_position_only_20261015_v1'))
from four_camera import nominal_transforms
LOCAL=nominal_transforms(.18197,.32506,.11503)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,v):Path(p).write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def load(p):return json.loads(Path(p).read_text())
def camera_vote(world,T,depth,valid,cad):
    q=(world-T[:3,3])@(T[:3,:3]@AX)
    z=q[:,2]; uv=np.zeros((len(q),2)); front=np.isfinite(q).all(1)&(z>=.5)&(z<=3)
    uv[front]=q[front,:2]/z[front,None]*F+np.array([319.5,239.5])
    inside=front&(uv[:,0]>=0)&(uv[:,0]<=639)&(uv[:,1]>=0)&(uv[:,1]<=479)
    ix=np.rint(np.clip(uv[:,0],0,639)).astype(int);iy=np.rint(np.clip(uv[:,1],0,479)).astype(int)
    pixels=iy*640+ix; observed=depth[iy,ix]
    usable=inside&~cad[iy,ix]&valid[iy,ix]&np.isfinite(observed)&(observed>=.5)&(observed<=3)
    residual=np.zeros(len(world));residual[usable]=observed[usable]-z[usable]
    vote=np.zeros(len(world),np.int8) #0 unavailable,1 closer,2 compatible,3 farther
    vote[usable& (residual<-.2)]=1;vote[usable&(abs(residual)<=.2)]=2;vote[usable&(residual>.2)]=3
    return vote,pixels,z,observed

def pair_classes(votes):
    # Pair codes:0 unavailable;1 closer/occluded;2 compatible;3 both farther;4 mixed/partial.
    result=[]
    for a,b in PAIRS:
        x,y=votes[:,a],votes[:,b];c=np.full(len(x),4,np.int8)
        c[(x==0)&(y==0)]=0;c[(x==2)&(y==2)]=2;c[(x==3)&(y==3)]=3
        c[(x==1)|(y==1)]=1
        result.append(c)
    return np.stack(result,axis=1)
