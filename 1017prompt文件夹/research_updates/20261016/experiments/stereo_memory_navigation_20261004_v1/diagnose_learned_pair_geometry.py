"""Bounded post-rejection attribution on saved pairs, no inference or gate change."""
from pathlib import Path
import json,numpy as np,cv2
from feature_tracks import _point,triangulate
H=Path(__file__).resolve().parent;K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
def main():
 m=json.loads((H/'learned_feature_runs/manifest.json').read_text());counts={};records=[]
 for row in m['rows']:
  p=json.loads(Path(row['path']).read_text());seq=row['sequence'];frame=row['frame'];q=np.load(H/'learned_feature_runs'/seq/f'features_{frame:03d}.npz')
  for d in p['causal']['pair_diagnostics']:
   f,eye,t=d['source'];s=np.load(H/'learned_feature_runs'/seq/f'features_{f:03d}.npz')
   for (i,j),epi in zip(d['learned_pairs'],d['epipolar_px']):
    key=seq+'/'+('current' if t==p['time_s'] else 'past');c=counts.setdefault(key,{'learned_pairs':0,'epipolar_reject':0,'nonpositive_depth':0,'parallax_reject':0,'reprojection_reject':0,'sensitivity_reject':0,'passed':0});c['learned_pairs']+=1
    if epi>1.5:c['epipolar_reject']+=1;continue
    a=q['left_xy'][i];b=s[eye+'_xy'][j];P=q['left_pose'];S=s[eye+'_pose'];point,lam,rays=_point(a,b,P,S,K);theta=float(np.degrees(np.arccos(np.clip(rays[0]@rays[1],-1,1))))
    if min(lam)<=0:c['nonpositive_depth']+=1;continue
    if theta<.5:c['parallax_reject']+=1;continue
    result=triangulate(a,b,P,S,K)
    if result is None:c['reprojection_reject']+=1;continue
    if result['sigma_m']>.1:c['sensitivity_reject']+=1
    else:c['passed']+=1
    records.append(dict(sequence=seq,frame=frame,source=d['source'],pair=[i,j],parallax_deg=theta,epipolar_px=epi,sigma_m=result['sigma_m'],world=result['world']))
 # Visual actual native pair and extracted/retained keypoints; no synthetic image.
 strips=[]
 for seq,i in [('e1',307),('hard_0_e1_0004',20)]:
  rgb=np.load(H/'learned_feature_inputs'/seq/f'rgb_{i:03d}.npz');features=np.load(H/'learned_feature_runs'/seq/f'features_{i:03d}.npz');images=[]
  for eye in ['left','right']:
   image=cv2.cvtColor(rgb[eye+'_rgb'],cv2.COLOR_RGB2BGR)
   for x,y in features[eye+'_xy']:cv2.circle(image,(round(x),round(y)),3,(0,255,255),1)
   cv2.putText(image,f'{seq} {i} {eye} kept={len(features[eye+"_xy"])}',(8,25),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1);images.append(image)
  strips.append(np.concatenate(images,axis=1))
 cv2.imwrite(str(H/'learned_feature_actual_keypoints.png'),np.concatenate(strips,axis=0))
 (H/'learned_feature_geometry_diagnostic.json').write_text(json.dumps(dict(counts=counts,post_rejection=True,no_new_predictions=True,thresholds_unchanged=True,rows=records),indent=2)+'\n');print(json.dumps(counts,indent=2))
if __name__=='__main__':main()
