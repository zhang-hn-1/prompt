"""One presealed archived-data geometry experiment, zero physics/control changes."""
from pathlib import Path
import json,hashlib,datetime,time,collections
import numpy as np
from provenance_grid import ProvenanceGrid
from reference_continuity_audit import reference_geometry_status
H=Path(__file__).resolve().parent;O=H/'depth_local_curve_run'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 started=datetime.datetime.now(datetime.timezone.utc).isoformat();p=json.loads((H/'reference_continuity_protocol.json').read_text());assert p['sealed_utc']<started and not (H/'reference_continuity_assessment.json').exists()
 for d in [p['source_sha256'],p['input_sha256']]:
  for f,s in d.items():assert sha(f)==s,f
 records=json.loads((O/'policy_records.json').read_text());a=np.load(O/'episode_arrays.npz');xy=a['positions_world_xy'];rows=[];last=None
 for i,r in enumerate(records):
  if not r['local']['local_plan_created']:continue
  g=np.load(O/'maps'/f'frame_{i:03d}.npz');assert float(g['time_s'])==r['local']['time_s'];assert (g['hazard_time'][g['labels']==1]<=float(g['time_s'])+1e-6).all()
  row=dict(frame=i,time_s=r['local']['time_s'],previous_install_frame=last,current_position_xy=xy[i].tolist(),current_target=r['local']['plan']['local_target'])
  if last is None:row['status']='not_applicable_first_reference'
  else:
   path=np.load(O/'local_plans'/f'frame_{i-1:03d}.npz')['pose'][:,:2];initial=np.load(O/'maps'/f'frame_{last:03d}.npz')
   row['previous_remaining']=reference_geometry_status(xy[i],path,g['labels'],g['origin'],float(g['resolution']))
   row['at_installation']=reference_geometry_status(xy[last],path,initial['labels'],initial['origin'],float(initial['resolution']))
   old_hazard=initial['labels']==1;new_hazard=g['labels']==1;added=new_hazard&~old_hazard
   row['new_hazard_cells_since_installation']=int(added.sum());row['new_hazard_world_xy']=(g['origin']+np.argwhere(added)*float(g['resolution'])).tolist();row['status']='measured'
  rows.append(row);last=i
 assert len(rows)==58 and [x['frame'] for x in rows]==list(range(60,350,5))
 out=H/'reference_continuity_routes';out.mkdir(exist_ok=False);counter=[]
 for f in p['switch_frames']:
  old=f-5;real_goal=np.load(O/'policy_inputs'/f'frame_{f:03d}.npz')['pointgoal_base_xy'];yaw=float(a['yaw_world_rad'][f]);c,s=np.cos(yaw),np.sin(yaw);goal=xy[f]+np.array([[c,-s],[s,c]])@real_goal
  for mf in [old,f]:
   raw=np.load(O/'maps'/f'frame_{mf:03d}.npz');grid=ProvenanceGrid(raw['origin'],shape=raw['labels'].shape,resolution=float(raw['resolution']));grid.labels=raw['labels'].copy();before=grid.labels.copy()
   for pf in [old,f]:
    tick=time.perf_counter();route,status=grid.plan(xy[pf],goal);assert np.array_equal(grid.labels,before)
    record=dict(event_frame=f,map_frame=mf,position_frame=pf,status=status,seconds=time.perf_counter()-tick,goal_xy=goal.tolist(),route_xy=None,target_xy=None,route_length_m=None,nominal_geometry_only=True,physical_outcome_available=False)
    if route is not None:
     arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(route,axis=0),axis=1))];idx=min(len(route)-1,int(np.searchsorted(arc,2.4)));target=route[idx];record.update(route_xy=route.tolist(),target_xy=target.tolist(),route_length_m=float(arc[-1]));np.savez_compressed(out/f'event_{f}_map_{mf}_pose_{pf}.npz',route_xy=route,target_xy=target,goal_xy=goal)
     if mf==f and pf==f:np.testing.assert_allclose(target,records[f]['local']['plan']['local_target'],rtol=0,atol=1e-12);np.testing.assert_equal(route,records[f]['local']['plan']['route_xy'])
    counter.append(record)
 assert len(counter)==8
 switches=[x for x in rows if x['frame'] in p['switch_frames']];recommend=all(x['previous_remaining']['retention_necessary_screen_pass'] for x in switches)
 counts=collections.Counter(reason for r in rows if r['status']=='measured' for reason in r['previous_remaining']['reject_reasons'])
 result=dict(saved_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),started_utc=started,protocol_sha256=sha(H/'reference_continuity_protocol.json'),rows=rows,counterfactuals=counter,replan_events=len(rows),measured_previous_references=57,grid_searches=len(counter),retention_proposal_necessary_screen_pass=recommend,screen_pass_frames=[r['frame'] for r in rows if r['status']=='measured' and r['previous_remaining']['retention_necessary_screen_pass']],reject_reason_counts=dict(counts),physical_control_changed=False,new_simulation=False,nominal_geometry_only=True,whole_research_complete=False)
 (H/'reference_continuity_assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ['rows','counterfactuals']},indent=2));print('SWITCHES',json.dumps(switches,indent=2));print('FACTORIAL',json.dumps([{k:v for k,v in r.items() if k!='route_xy'} for r in counter],indent=2))
if __name__=='__main__':main()
