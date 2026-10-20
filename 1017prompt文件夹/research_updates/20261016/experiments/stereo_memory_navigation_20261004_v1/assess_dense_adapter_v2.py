"""Labels decoded only after ALL fresh B0+candidate predictions are saved."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from stereo_geometry import FOCAL,BASELINE
from dense_adapter_metrics import source_scores
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
 p=json.loads((H/'dense_adapter_fit_v2_protocol.json').read_text());out=H/'dense_adapter_v2_predictions';manifest=json.loads((out/'manifest.json').read_text());assert manifest['labels_not_loaded'] and len(manifest['rows'])==130
 assert {(x['arm'],x['frame']) for x in manifest['rows']}=={(arm,i) for arm in ('b0','adapter') for i in range(0,321,5)}
 for f,d in p['source_sha256'].items():assert sha(f)==d,f
 for r in manifest['rows']:assert sha(r['path'])==r['sha256']
 masks=np.load(p['mask']);y,x=np.indices((480,640));rays=np.stack([(x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(x.shape)],axis=-1);sums={};rows=[]
 for frame in range(0,321,5):
  truth=np.load(H/'dense_adapter_heldout_run/frames'/f'frame_{frame:03d}.npz');z=truth['left_depth_m'];pose=truth['left_camera_usd_world'];root=truth['root_position_world_m']
  with np.errstate(invalid='ignore'):world=pose[:3,3]+(rays*z[...,None])@(pose[:3,:3]@np.diag([1,-1,-1])).T
  valid=np.isfinite(z)&(z>=.5)&(z<=3)&~masks['left']&(np.linalg.norm(world-root,axis=-1)>=.55)
  vr=y-FOCAL*BASELINE/np.maximum(z,1e-9);iv=np.rint(np.nan_to_num(vr,nan=-1,posinf=-1,neginf=-1)).astype(int);ix=np.clip(iv,0,479);rz=truth['right_depth_m'][ix,x]
  with np.errstate(invalid='ignore'):overlap=(vr>=0)&(vr<=479)&~masks['right'][ix,x]&np.isfinite(rz)&(np.abs(rz-z)<=.03+.01*z)
  groups=dict(body_all=valid&(world[...,2]>=.05)&(world[...,2]<=1.5),body_overlap=valid&overlap&(world[...,2]>=.05)&(world[...,2]<=1.5),floor_all=valid&(np.abs(world[...,2])<=.05),floor_overlap=valid&overlap&(np.abs(world[...,2])<=.05))
  for arm in ('b0','adapter'):
   pred=np.load(out/arm/f'frame_{frame:03d}.npz');assert np.shape(pred['depth_m'])==(480,640);assert np.array_equal(np.isfinite(pred['depth_m']),pred['confidence'])
   with np.errstate(invalid='ignore'):err=np.abs(pred['depth_m']-z)*np.linalg.norm(rays,axis=-1)
   err=np.where(np.isfinite(err),err,np.inf);row=dict(arm=arm,frame=frame,time_s=float(truth['time_s']),groups={})
   for name,sel in groups.items():
    sample=sel&(y%2==0)&(x%2==0);e=err[sample];row['groups'][name]=source_scores(e);sums.setdefault(arm,{}).setdefault(name,[]).append(e)
   rows.append(row)
 pooled={arm:{name:source_scores(np.concatenate(errors)) for name,errors in sub.items()} for arm,sub in sums.items()}
 gate={arm:dict(pooled=pooled[arm]['body_overlap']['gate_pass'],eligible_frames=sum(r['groups']['body_overlap']['adequate'] for r in rows if r['arm']==arm),frames_passing=sum(r['groups']['body_overlap']['gate_pass'] for r in rows if r['arm']==arm),insufficient_frames=sum(not r['groups']['body_overlap']['adequate'] for r in rows if r['arm']==arm)) for arm in ('b0','adapter')}
 for arm,g in gate.items():g['candidate_pass']=bool(g['pooled'] and g['eligible_frames']>0 and g['eligible_frames']==g['frames_passing'])
 result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),predictions_saved_utc=manifest['saved_utc'],pooled=pooled,gate=gate,rows=rows,adapter_admitted=gate['adapter']['candidate_pass'],source_scope='whole independent initialconditionE2 sameenvironment; RGB-onlysource; labels fromactualrenderer offline; not memory/freevolume/pose/nav',whole_research_complete=False)
 (out/'assessment.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
if __name__=='__main__':main()
