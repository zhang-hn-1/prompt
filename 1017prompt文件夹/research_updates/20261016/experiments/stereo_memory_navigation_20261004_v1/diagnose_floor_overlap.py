from pathlib import Path
import json,datetime,hashlib
import numpy as np
from multiview_rgb import optical_rotation
from past_multiview import measure_view
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=json.loads((H/'floor_overlap_diagnostic_protocol.json').read_text());masks=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz');yy,xx=np.mgrid[:480,:640];f=584.8888888888889;cx=319.5;cy=239.5;rays=np.stack([(xx-cx)/f,(yy-cy)/f,np.ones((480,640))],axis=-1);rows=[];inputs={}
 for i in p['reference_frames']:
  file=H/'rolling_online_run/offline_truth'/f'frame_{i:03d}.npz';truth=np.load(file);inputs[str(file)]=sha(file);pose=truth['left_camera_usd_world'];z=truth['left_depth_m']
  with np.errstate(invalid='ignore'):points=pose[:3,3]+(rays*z[...,None])@optical_rotation(pose).T
  query=np.isfinite(z)&(z>=.5)&(z<=3)&~masks['left']&(abs(points[...,2])<=.05);pts=points[query];n=len(pts);sources=[]
  for offset in [0]+p['past_offsets']:
   if i-offset<0:continue
   file=H/'rolling_online_run/offline_truth'/f'frame_{i-offset:03d}.npz';s=np.load(file);inputs[str(file)]=sha(file)
   for eye in p['eyes']:
    if offset==0 and eye=='left':continue
    v=measure_view(pts,s[eye+'_camera_usd_world'],s[eye+'_depth_m'],masks[eye],f,cx,cy,.01);a=pts-pose[:3,3];b=pts-s[eye+'_camera_usd_world'][:3,3];angle=np.degrees(np.arctan2(np.linalg.norm(np.cross(a,b),axis=1),np.sum(a*b,axis=1)));good=v['visible']&(angle>=.5)
    sources.append(dict(offset=offset,eye=eye,visible=int(v['visible'].sum()),visible_parallax=int(good.sum()),flag=good))
  old=[s['flag'] for s in sources if (s['offset']==0 and s['eye']=='right') or s['offset']==5];new=[s['flag'] for s in sources if s['offset'] in [0,1,2]]
  count=lambda fs:np.sum(fs,axis=0) if fs else np.zeros(n,int)
  oc=count(old);nc=count(new);rows.append(dict(frame=i,queries=n,sources=[{k:v for k,v in s.items() if k!='flag'} for s in sources],old_any=int((oc>=1).sum()),old_two=int((oc>=2).sum()),recent_any=int((nc>=1).sum()),recent_two=int((nc>=2).sum()),recent_two_fraction=float((nc>=2).mean()) if n else None))
 result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),protocol_sha256=sha(H/'floor_overlap_diagnostic_protocol.json'),source_sha256=sha(__file__),input_sha256=inputs,rows=rows,new_predictions=0,new_fits=0,new_physics=0,old_candidates_rejected=True)
 (H/'floor_overlap_diagnostic.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(rows,indent=2))
if __name__=='__main__':main()
