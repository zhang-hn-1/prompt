from pathlib import Path
import sys,json,hashlib,datetime,itertools
import numpy as np
from reference import build_cost,aggregate,decode
D=Path(__file__).resolve().parent;W=D.parents[1];sys.path.insert(0,str(W/'experiments/observed_support_aggregation_reference_20261018_v1'))
from kernel import aggregate as scalar
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def scalar_field(im,y,x,ch,gradient):
 H,W=im.shape[:2]
 def v(yy,xx):return int(im[yy,xx,ch]) if 0<=yy<H and 0<=xx<W else 0
 if not gradient:return v(y,x)
 g=2*(v(y,x+1)-v(y,x-1))+v(y-1,x+1)-v(y-1,x-1)+v(y+1,x+1)-v(y+1,x-1)
 return min(31,max(-31,g))+31

def interval(im,y,x,ch,gradient):
 v=scalar_field(im,y,x,ch,gradient);l=(v+scalar_field(im,y,x-1,ch,gradient))//2;u=(v+scalar_field(im,y,x+1,ch,gradient))//2
 return v,min(v,l,u),max(v,l,u)

def unary_scalar(a,b,y,x,d):
 val=0
 for yy in range(y-2,y+3):
  for xx in range(x-2,x+3):
   for grad in [True,False]:
    for ch in range(3):
     u,u0,u1=interval(a,yy,xx,ch,grad);v,v0,v1=interval(b,yy,xx-d,ch,grad)
     bt=min(max(0,u-v1,v0-u),max(0,v-u1,u0-v));val+=bt if grad else bt//4
 return val

def main():
 cfg=json.loads((D/'protocol.json').read_text())
 for sec in ['code_sha256','input_sha256']:
  for p,h in cfg[sec].items():assert sha(p)==h,p
 rng=np.random.default_rng(20261018);checks={};count=0
 for i in range(4):
  c=rng.integers(0,4001,(3,5,4)).astype(float);m=rng.random(c.shape)>.25
  a,ap=aggregate(c,m,True);b,bp=scalar(c,m)
  assert a.tobytes()==b.tobytes()
  for n in ap:assert ap[n].tobytes()==bp[n].tobytes()
 checks['vectorized_scalar_fourvolumes_all3paths_byteexact']=True
 a=rng.integers(0,256,(11,15,3),dtype=np.uint8);b=rng.integers(0,256,a.shape,dtype=np.uint8);ds=np.array([-3,0,4]);before=a.tobytes(),b.tobytes();c,m=build_cost(a,b,ds)
 for y,x,k in itertools.product([0,3,5,10],[0,4,8,14],range(3)):
  expected=unary_scalar(a,b,y,x,int(ds[k]));assert c[y,x,k]==expected,(y,x,k,c[y,x,k],expected);count+=1
  ok=3<=y<8 and 4<=x<11 and 4<=x-ds[k]<11;assert bool(m[y,x,k])==ok
 checks['RGB_BT_and_window_48scalarcosts_exact']=True;checks['per_hypothesis_support_48points_exact']=True;checks['RGB_inputs_immutable']=before==(a.tobytes(),b.tobytes())
 # Constant field must not manufacture an observation from a tie.
 z=np.zeros((11,15,3),np.uint8);cc,mm=build_cost(z,z,np.array([0,1]));t,_=aggregate(cc,mm);dd=decode(t,np.array([0,1]));checks['flat_valid_tie_UNKNOWN']=bool(np.isnan(dd[5,6]))
 t=np.array([[[100.,100.,900.],[0.,100.,900.],[10.,100.,11.]]]);dec=decode(t,np.array([0,1,2]));checks['decoder_tie_ratio_and_integer_sign']=bool(np.isnan(dec[0,0]) and dec[0,1]==0 and np.isnan(dec[0,2]))
 # Synthetic exactshift, no geometry/GT/radius fitting.
 a=rng.integers(0,256,(15,23,3),dtype=np.uint8);b=np.zeros_like(a);b[:,:-3]=a[:,3:];cc,mm=build_cost(a,b,np.arange(7));checks['known_positive_shift_zero_supported_cost']=bool(mm[7,12,3] and cc[7,12,3]==0)
 cc,mm=build_cost(b,a,np.arange(-6,1));checks['known_reverse_shift_zero_supported_cost']=bool(mm[7,9,3] and cc[7,9,3]==0)
 assert all(checks.values()),checks
 (D/'fixtures_result.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),complete=True,checks=checks,scalar_BT_checks=count,vectorized_scalar_volumes=4,real_RGB_calls=0,GT_reads=0),indent=2)+'\n');print('FIXTURES',checks,flush=True)
if __name__=='__main__':main()
