import bootstrap
from bootstrap import D,W
import json,hashlib,numpy as np
from types import SimpleNamespace as N
from cursor_guard import reference_geometry_status,leg_interval
from reference_continuity_audit import reference_geometry_status as old
from continuous_cell_tube_v1 import continuous_cell_tube_screen
R=W/'experiments/source_confidence_calibration_20261012_v1/data_e1_run';rows=json.loads((R/'policy_records.json').read_text());plans={};result=[];hashes={}
class Grid:
 def __init__(self,z):self.labels=z['labels'];self.origin=z['origin'];self.resolution=float(z['resolution'])
 def cells(self,xy):return np.floor((np.asarray(xy)-self.origin)/self.resolution+.5).astype(int)
for i,r in enumerate(rows):
 v=r['local_reference_version']
 if v and v not in plans:
  p=R/'local_plans'/f'frame_{i:03d}.json';plans[v]=json.loads(p.read_text());hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
 d=r.get('reference_decision') or {};g=d.get('guard')
 if not g or abs(d.get('measured_time_s',-99)-r['time_s'])>1e-6 or i==0:continue
 plan=plans[rows[i-1]['local_reference_version']];path=np.array([[a['x_m'],a['y_m']] for a in plan['waypoints']]);xy=np.array(g['bridge_xy'][0]);mp=R/'maps'/f'frame_{i:03d}.npz';grid=Grid(np.load(mp));snap=grid.labels.tobytes();state=d['state_before']
 tr=N(_reference=[N(**w) for w in plan['waypoints']],_path_index=state['path_index'],_active_drive_direction=state['active_direction'],_pending_drive_direction=state['pending_direction'],_braking_state=state['braking_state'])
 b=old(xy,path,grid.labels,grid.origin,grid.resolution)
 assert b==g,(i,'old guard mismatch')
 eligible=leg_interval(tr);c=reference_geometry_status(xy,path,grid.labels,grid.origin,grid.resolution,segment_interval=eligible)
 armed=d['tube_viability_armed_at_boundary'];bt=continuous_cell_tube_screen(grid,b) if b['retention_necessary_screen_pass'] and armed else None;ct=continuous_cell_tube_screen(grid,c) if c['retention_necessary_screen_pass'] and armed else None
 bk=b['retention_necessary_screen_pass'] and (not armed or bt['pass']);ck=c['retention_necessary_screen_pass'] and (not armed or ct['pass']);assert bk==(d['decision']=='retain'),(i,'old decision mismatch');assert snap==grid.labels.tobytes()
 q=dict(frame=i,reference=rows[i-1]['local_reference_version'],state=state,eligible_interval=eligible,baseline_guard=b,candidate_guard=c,baseline_tube=bt,candidate_tube=ct,baseline_retain=bk,candidate_retain=ck,changed_decision=bk!=ck,map_unchanged=True);result.append(q);hashes[str(mp)]=hashlib.sha256(mp.read_bytes()).hexdigest()
 if bk!=ck:print('changed',i,bk,ck,'clearance',None if bt is None else bt['minimum_continuous_cell_clearance_m'],None if ct is None else ct['minimum_continuous_cell_clearance_m'],flush=True)
for p in [R/'policy_records.json']:hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
gate=any(x['frame'] in [660,815] and not x['baseline_retain'] and x['candidate_retain'] for x in result)
o=dict(pass_gate=gate,boundaries=len(result),changed_decisions=sum(x['changed_decision'] for x in result),original_guard_and_decisions_exact=True,queries=result,input_sha256=hashes,scope='saved factual state sequence only; after earliest changed decision these are conditional shadow checks, not candidate trajectory')
(D/'replay.json').write_text(json.dumps(o,indent=2)+'\n');print('PASS',gate,'boundaries',len(result))
