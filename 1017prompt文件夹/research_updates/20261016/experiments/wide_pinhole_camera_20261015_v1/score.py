from pathlib import Path
import sys,json,hashlib,numpy as np
from pxr import Usd,UsdGeom
D=Path(__file__).resolve().parent;out=D/(sys.argv[1] if len(sys.argv)>1 else 'native_recovery_run');seal=json.loads((out/'capture_seal.json').read_text());assert seal['complete'] and len(seal['rows'])==8;stage=Usd.Stage.Open(str(out/'stage_root_initial.usda'));cache=UsdGeom.XformCache();objects=[]
for p in stage.Traverse():
 if p.IsA(UsdGeom.Cylinder):
  c=UsdGeom.Cylinder(p);T=np.array(cache.GetLocalToWorldTransform(p)).T;assert np.allclose(T[:3,:3],np.eye(3));objects.append((str(p.GetPath()),T[:3,3],float(c.GetRadiusAttr().Get()),float(c.GetHeightAttr().Get())))
rows=[];scored=[];f=584.8888888888889/3
for rr in seal['rows']:
 p=out/'frames'/rr['frame'];assert hashlib.sha256(p.read_bytes()).hexdigest()==rr['sha256'];a=np.load(p)
 for eye in [0,1]:
  T=a[f'eye{eye}_camera_usd_world_label'];o=T[:3,3];z=a[f'eye{eye}_depth_label']
  for y,x in [(240,320),(200,320),(280,320),(240,280),(240,360)]:
   d=T[:3,:3]@np.diag([1,-1,-1])@np.array([(x-319.5)/f,(y-239.5)/f,1.]);hits=[]
   for name,c,r,h in objects:
    v=o-c;A=d[:2]@d[:2];B=2*(v[:2]@d[:2]);C=v[:2]@v[:2]-r*r;disc=B*B-4*A*C
    if disc>=0:
     for t in [(-B-np.sqrt(disc))/(2*A),(-B+np.sqrt(disc))/(2*A)]:
      if t>.01 and abs(v[2]+t*d[2])<=h/2:hits.append((float(t),name))
   expected=min(hits)[0] if hits else None;eligible=expected is not None and .5<=expected<=3;err=abs(float(z[y,x])-expected) if eligible and np.isfinite(z[y,x]) else None;row=dict(frame=rr['frame'],eye=eye,pixel=[y,x],expected_cylinder_depth=expected,saved_depth=float(z[y,x]) if np.isfinite(z[y,x]) else None,scored=eligible,error_m=err);rows.append(row)
   if eligible:scored.append(row)
passed=sum(x['error_m'] is not None and x['error_m']<=.02 for x in scored);result=dict(total_fixed_ray_queries=len(rows),scored=len(scored),correct_within_02m=passed,scored_fraction=len(scored)/len(rows),conditional_pass_fraction=passed/max(1,len(scored)),pass_gate=len(scored)>=16 and passed/max(1,len(scored))>=.9,rows=rows,scope='Fixed 5 pixels per eye per native frame; same-runtime exported scene stage cylinder ray/depth identity, not full depth precision or monocular output. Only expected finite .5-3 eligible, no-hit/unscored unknown. Does not certify complete visibility/free/long memory/navigation.')
(D/'ray_qualification.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print({k:v for k,v in result.items() if k!='rows'},flush=True)
