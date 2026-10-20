"""Postseal causal development audit; GT labels never feed any candidate."""
from pathlib import Path
import numpy as np,json,datetime
from scipy.spatial import cKDTree
H=Path(__file__).resolve().parent;D=H/'connected_surface_memory_v1';O=H/'qualified_hazard_navigation_run_v1';seed=np.load(O/'sensor_packets/source_180.npz')['world'];seed=seed[(seed[:,2]>.05)&(seed[:,2]<=1.5)];hist=[(float(np.load(O/'sensor_packets/source_180.npz')['time_s']),seed)];rows=[]
for i in [235,245,255]:
 packets=[np.load(D/f'frame_{i}_{eye}.npz') for eye in ['left','right']];t=float(packets[0]['current_time_s']);w=np.concatenate([p['world'] for p in packets]);v=np.concatenate([p['bilinear_patch_contract_valid'] for p in packets]);accepted=np.concatenate([p['F1_connected_accepted'] for p in packets]);hist=[(a,b) for a,b in hist if t-a<=10.];anchors=np.concatenate([b for a,b in hist]);mind=cKDTree(anchors).query(w)[0];node=np.flatnonzero(v);tree=cKDTree(w[v]);seen=np.zeros(len(node),bool);components=[]
 for k in range(len(node)):
  if seen[k]:continue
  seen[k]=True;stack=[k];comp=[]
  while stack:
   j=stack.pop();comp.append(node[j])
   for z in tree.query_ball_point(w[node[j]],.1):
    if not seen[z]:seen[z]=True;stack.append(z)
  comp=np.array(comp);components.append(dict(points=len(comp),min_distance_to_old_m=float(mind[comp].min()),left_points=int((comp<len(packets[0]['world'])).sum()),accepted_points=int(accepted[comp].sum())))
 for eye,p in zip(['left','right'],packets):
  q=p['world'];uvs=[p['current_uv'],*p['past_uv']];errors=[]
  for frame,uv in zip([i,*p['source_frames']],uvs):
   gt=np.load(O/'offline_truth'/f'frame_{frame:03d}.npz');x,y=np.rint(uv).astype(int).T;T=gt[eye+'_camera_usd_world'];z=gt[eye+'_depth_m'][y,x];ray=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))]
   with np.errstate(invalid='ignore'):tw=T[:3,3]+(ray*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;errors.append(np.linalg.norm(tw-q,axis=1))
  e=np.asarray(errors).T;finite=np.isfinite(e).all(1);wrong=finite&np.any(e>.1,axis=1);a=p['F1_connected_accepted'];okay=finite&~wrong& a;near=cKDTree(q[okay]).query(q[wrong&a])[0] if okay.any() else np.array([]);part=p['bilinear_patch_contract_valid'];md=cKDTree(anchors).query(q[part])[0] if part.any() else np.array([])
  rows.append(dict(frame=i,eye=eye,valid_proposals=int(part.sum()),minimum_distance_to_prior_anchor_m=float(md.min()) if len(md) else None,accepted_finite_wrong=int((wrong&a).sum()),wrong_within_10cm_of_current_correct=int((near<=.1).sum()),accepted_wrong_to_correct_distance_median=float(np.median(near)) if len(near) else None))
 hist.append((t,w[accepted]));rows.append(dict(frame=i,joint_components=sorted(components,key=lambda c:-c['points'])))
path=D/'component_audit_v1.json';assert not path.exists();path.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),postseal_GT_audit_only=True,no_gate_or_candidate_changes=True,rows=rows),indent=2)+'\n');print(rows)
