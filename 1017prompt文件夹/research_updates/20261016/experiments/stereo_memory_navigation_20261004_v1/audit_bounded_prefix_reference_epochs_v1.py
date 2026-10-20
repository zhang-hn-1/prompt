"""Postclosed full-reference epochs; no replay, action or new acceptance gate."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from current_reference_epoch_geometry_v1 import project
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 result={};inputs={}
 for name in ('b0','candidate'):
  d=H/f'bounded_prefix_e1_{name}_run_v1';rf=d/'policy_records.json';sf=d/'states.json';mf=d/'artifact_manifest_post_close.json';seal=json.loads(mf.read_text())['artifact_sha256']
  for f in (rf,sf):assert seal[str(f)]==sha(f)
  rows=json.loads(rf.read_text());states=json.loads(sf.read_text());epochs=[];current=None;prior=None
  for row in rows:
   waypoints=row['current_reference_waypoints_after_act'];identity=hashlib.sha256(json.dumps(waypoints,sort_keys=True,separators=(',',':')).encode()).hexdigest()
   if identity!=prior:
    current=dict(reference_sha256=identity,first_frame=row['frame'],last_frame=row['frame'],waypoint_count=len(waypoints),raw_waypoint_direction_counts={k:sum(w['drive_direction']==k for w in waypoints) for k in ('forward','reverse')},geometry_samples=[]);epochs.append(current);prior=identity
   current['last_frame']=row['frame'];f=row['frame'];assert states[f]['frame']==f if 'frame' in states[f] else True
   if len(waypoints)<2:continue
   path=[[w['x_m'],w['y_m']] for w in waypoints];np.testing.assert_array_equal(np.asarray(path),np.asarray(row['installed_reference_xy_after_act']))
   x=project(path,states[f]['position'][:2]);x.update(frame=f,time_s=states[f]['time_s']);current['geometry_samples'].append(x)
  for epoch in epochs:
   g=epoch['geometry_samples'];epoch['request_interval_s']=(epoch['last_frame']-epoch['first_frame']+1)*.1
   if g:epoch.update(entry=g[0],exit=g[-1],minimum_endpoint_distance_m=min(x['endpoint_distance_m'] for x in g),cross_RMS_m=float(np.sqrt(np.mean([x['distance_m']**2 for x in g]))),cross_peak_m=max(x['distance_m'] for x in g),ambiguous_samples=sum(x['ambiguous'] for x in g),projection_arc_change_interval_m=[g[-1]['arc_min_m']-g[0]['arc_max_m'],g[-1]['arc_max_m']-g[0]['arc_min_m']])
  result[name]=epochs
  for f in (rf,sf,mf):inputs[str(f)]=sha(f)
 out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),epochs=result,input_sha256=inputs,source_sha256={str(p):sha(p) for p in (Path(__file__).resolve(),H/'current_reference_epoch_geometry_v1.py',H/'test_current_reference_epoch_geometry_v1.py',H/'current_reference_epoch_geometry_scope_v1.json')},notes=['Each frozen reference epoch is evaluated separately; never sum arc progress across rebuilt references.','Each sample is current pre-act measured XY paired with the full installed reference after that one call; reference installation is a decision, not proof of prior exposure.','Last epoch sample is the final request pre-state, not the final postphysics arrival state. Original navigation scoring remains authoritative.','Arc intervals preserve self-overlap ambiguity; no policy path_index or heading chooses a convenient projection.','Raw waypoint direction labels are logged content, not independently inferred actual movement.'],no_new_solver_physics_fit=True,not_online_interface_validation=True,no_new_gate=True,whole_mission_complete=False)
 p=H/'bounded_prefix_current_reference_epochs_v1.json';assert not p.exists();p.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({a:[{k:e.get(k) for k in ('first_frame','last_frame','waypoint_count','raw_waypoint_direction_counts','minimum_endpoint_distance_m','cross_RMS_m','ambiguous_samples','projection_arc_change_interval_m')} for e in es] for a,es in result.items()}))
if __name__=='__main__':main()
