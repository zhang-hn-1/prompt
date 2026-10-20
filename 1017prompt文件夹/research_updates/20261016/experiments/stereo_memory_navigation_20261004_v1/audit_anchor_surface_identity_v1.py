"""Postseal physical-point identity diagnosis using old teachers only."""
from pathlib import Path
import json,hashlib
import numpy as np
H=Path(__file__).resolve().parent;O=H/'anchor_evidence_development_v1';D=H/'minimum_prefix_transition_navigation_run_v1'
rows=[]
for r in json.loads((O/'teacher_sampling_sensitivity_v2.json').read_text())['rows']:
 q=np.asarray(r['query_xy']);x,y=np.rint(q).astype(int);cur=np.load(D/'frames'/f"frame_{r['frame']:03d}.npz");T=cur['left_camera_usd_world'];z=float(cur['left_depth_m'][y,x]);w=T[:3,3]+(T[:3,:3]@np.diag([1,-1,-1]))@(np.r_[(q-[319.5,239.5])/584.8888888888889,1]*z)
 pred=next(e['original'] for p in json.loads((O/'predictions.json').read_text()) if p['frame']==r['frame'] for e in p['geometries'] if e['original']['source_frame']==r['source_frame'] and e['original']['query_xy']==r['query_xy']);s=np.asarray(pred['source_xy']);sx,sy=np.rint(s).astype(int);past=np.load(D/'frames'/f"frame_{r['source_frame']:03d}.npz");S=past['left_camera_usd_world'];sz=float(past['left_depth_m'][sy,sx]);dist=None
 if np.isfinite(sz) and sz>0:
  sw=S[:3,3]+(S[:3,:3]@np.diag([1,-1,-1]))@(np.r_[(s-[319.5,239.5])/584.8888888888889,1]*sz);dist=float(np.linalg.norm(sw-w))
 rows.append(dict(frame=r['frame'],source_frame=r['source_frame'],query_xy=r['query_xy'],old_contained=r['samples'][0]['contained'],past_projection_pixel_error=r['samples'][0]['past_pixel_error_max'],GT_projected_past_depth_delta_m=r['samples'][0]['past_depth_delta_m'],GT_current_vs_matched_past_world_distance_m=dist))
summary={}
for k,mask in [('contained',lambda r:r['old_contained']),('all_misses',lambda r:not r['old_contained']),('large_misses_over4px',lambda r:not r['old_contained'] and r['past_projection_pixel_error']>4)]:
 subset=[r for r in rows if mask(r)];d=[r['GT_current_vs_matched_past_world_distance_m'] for r in subset if r['GT_current_vs_matched_past_world_distance_m'] is not None];pd=[abs(r['GT_projected_past_depth_delta_m']) for r in subset if r['GT_projected_past_depth_delta_m'] is not None]
 summary[k]=dict(pair_count=len(subset),matched_past_valid_count=len(d),world_distance_median_m=float(np.median(d)) if d else None,world_distance_max_m=max(d) if d else None,projected_past_valid_count=len(pd),projected_depth_delta_abs_median_m=float(np.median(pd)) if pd else None,projected_depth_delta_abs_max_m=max(pd) if pd else None)
(O/'surface_identity_diagnosis_v1.json').write_text(json.dumps(dict(summary=summary,rows=rows,teacher_after_rejection_only=True,no_source_or_gate_changes=True,not_isolation_of_reflectance_or_render_noise=True,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2)+'\n');print(json.dumps(summary,indent=2))
