"""Post-run truth-only endpoint diagnosis. Not online decision input."""
from pathlib import Path
import json,numpy as np
H=Path(__file__).resolve().parent;out={}
for arm in ('current','history'):
 O=H/('prior_rgb_'+arm+'_run');totals=dict(records=0,correct_3d_le_10cm=0,predicted_floor=0,predicted_hazard=0,hazard_over_true_floor=0,floor_over_true_nonfloor=0);frames=[];errors=[]
 for f in sorted((O/'sensor_packets').glob('*.npz')):
  p=np.load(f);i=int(p['frame']);truth=np.load(O/'offline_truth'/f'frame_{i:03d}.npz');q=p['query_pixel_id'];yy=q//640;xx=q%640;z=truth['left_depth_m'][yy,xx];pose=truth['left_camera_usd_world'];rays=np.c_[(xx-319.5)/584.8888888888889,(yy-239.5)/584.8888888888889,np.ones(len(q))];actual=pose[:3,3]+rays*z[:,None]@(pose[:3,:3]@np.diag([1,-1,-1])).T;finite=np.isfinite(actual).all(1);err=np.linalg.norm(p['world']-actual,axis=1);ph=(p['world'][:,2]>.05)&(p['world'][:,2]<=1.5);pf=(np.abs(p['world'][:,2])+p['sigma_m']<=.05)&(p['sigma_m']<=.05);truefloor=np.abs(actual[:,2])<=.05
  vals=dict(records=len(q),correct_3d_le_10cm=int((finite&(err<=.1)).sum()),predicted_floor=int(pf.sum()),predicted_hazard=int(ph.sum()),hazard_over_true_floor=int((ph&finite&truefloor).sum()),floor_over_true_nonfloor=int((pf&finite&~truefloor).sum()))
  for k in totals:totals[k]+=vals[k]
  errors.extend(err[finite].tolist());frames.append(dict(frame=i,**vals))
 rows=json.loads((O/'policy_records.json').read_text());reasons={}
 for r in rows:
  name=r['local']['reference_decision']['decision'] if r['local']['reference_decision'] else 'settling';reasons[name]=reasons.get(name,0)+1
 out[arm]=dict(source_totals=totals,conditional_endpoint_error_p90_m=float(np.percentile(errors,90)) if errors else None,source_frames=frames,reference_decision_blocks=reasons,scope='offline current-source truth, development diagnosis; cannot establish necessity or safety')
(H/'prior_rgb_navigation_source_diagnosis.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:{n:v for n,v in x.items() if n!='source_frames'} for k,x in out.items()},indent=2))
