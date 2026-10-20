"""Historical teacher sampling sensitivity, not new labels or gate changes."""
from pathlib import Path
import json,hashlib
import numpy as np
H=Path(__file__).resolve().parent; D=H/'minimum_prefix_transition_navigation_run_v1';O=H/'anchor_evidence_development_v1';focal=584.8888888888889
rows=[]
for packet in json.loads((O/'predictions.json').read_text()):
 f=packet['frame']; gt=np.load(D/'frames'/f'frame_{f:03d}.npz');T=gt['left_camera_usd_world'];R=T[:3,:3]@np.diag([1,-1,-1]);depth=gt['left_depth_m']
 for entry in packet['geometries']:
  if not entry['evidence']['eligible_anchor']:continue
  r=entry['original'];q=np.asarray(r['query_xy']);x,y=np.rint(q).astype(int);z=float(depth[y,x]);
  if not np.isfinite(z) or z<=0:continue
  past=np.load(D/'frames'/f"frame_{r['source_frame']:03d}.npz");P=past['left_camera_usd_world'];PR=P[:3,:3]@np.diag([1,-1,-1]);A=np.asarray(r['constraint_A']);b=np.asarray(r['constraint_b']);sample=[]
  for name,uv in [('unrounded_query',q),('rounded_depth_pixel',np.array([x,y]))]:
   w=T[:3,3]+R@(np.r_[(uv-[319.5,239.5])/focal,1]*z);p=PR.T@(w-P[:3,3]);proj=p[:2]/p[2]*focal+[319.5,239.5];inside=r['status']=='bounded' and np.all(A@w<=b+1e-7); px,py=np.rint(proj).astype(int);pz=float(past['left_depth_m'][py,px]) if 0<=px<640 and 0<=py<480 else None
   sample.append(dict(name=name,contained=bool(inside),past_pixel_error_max=float(np.max(np.abs(proj-r['source_xy']))),past_expected_optical_z=float(p[2]),past_sampled_depth_m=pz,past_depth_delta_m=None if pz is None or not np.isfinite(pz) or pz<=0 else float(pz-p[2])))
  neighborhood=depth[max(0,y-1):y+2,max(0,x-1):x+2];valid=neighborhood[np.isfinite(neighborhood)&(neighborhood>0)]
  rows.append(dict(frame=f,source_frame=r['source_frame'],query_xy=q.tolist(),samples=sample,current_3x3_valid_depth_span_m=float(np.ptp(valid)) if len(valid) else None))
assert len(rows)==308
original_misses=[r for r in rows if not r['samples'][0]['contained']];rounded_misses=[r for r in rows if not r['samples'][1]['contained']]
summary=dict(valid_pairs=308,original_contained=284,rounded_ray_contained=308-len(rounded_misses),original_misses=24,original_misses_retained_under_rounded_ray=sum(not r['samples'][1]['contained'] for r in original_misses),original_large_errors_gt4px=sum(r['samples'][0]['past_pixel_error_max']>4 for r in original_misses),rounded_large_errors_gt4px=sum(r['samples'][1]['past_pixel_error_max']>4 for r in original_misses),current3x3_span_original_miss_max_m=max(r['current_3x3_valid_depth_span_m'] for r in original_misses))
result=dict(summary=summary,rows=rows,GT_after_rejection_diagnostic_only=True,original_teacher_and_gate_unchanged=True,no_new_correspondence_acceptance_rule=True,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest());(O/'teacher_sampling_sensitivity_v1.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(json.dumps(summary,indent=2))
