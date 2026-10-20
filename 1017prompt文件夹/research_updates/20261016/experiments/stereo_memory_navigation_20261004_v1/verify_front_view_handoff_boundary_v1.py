"""Postclose factual handoff, input causality and request checks, not alternate-plan truth."""
from pathlib import Path
import json,hashlib,numpy as np
from tentative_point_memory import TentativePointMemory
from front_view_acquisition_v1 import FrontViewAcquisition
H=Path(__file__).resolve().parent;D=H/'front_view_handoff_navigation_run_v1'
assert json.loads((H/'front_view_handoff_navigation_status_v1.json').read_text())['status']=='finished'
seal=json.loads((D/'artifact_manifest_post_close.json').read_text())
for p,d in seal['artifact_sha256'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==d,p
states=json.loads((D/'states.json').read_text());sources=json.loads((D/'source_rows.json').read_text());rows=json.loads((D/'policy_records.json').read_text());req=np.load(D/'requests.npz')['requests'];by={s['frame']:s for s in sources};memory=TentativePointMemory();acq=FrontViewAcquisition();source=None;handoff=None;prefix=0
for i,row in enumerate(rows):
 t=states[i]['time_s']
 if i in by:
  source=by[i];p=np.load(D/'partial_observations'/f'frame_{i:03d}.npz');memory.update(dict(frame=i,time_s=t,world=p['world'],query_pixel_id=p['query_pixel_id'],partner_pixel_id=p['partner_pixel_id'],input_sha256=source['provenance']),t)
 src=dict(frame=source['frame'],time_s=source['time_s'],records=memory.at(t)['records'])
 action=acq.choose(t,states[i]['position'][:2],np.linalg.norm(states[i]['velocity'][:2]),src)
 assert row['front_view_acquisition']==acq.diagnostics
 if action is not None:
  np.testing.assert_array_equal(req[i],action);assert row['prefix_observer'] and not row['planner_act_called'] and not row['tracker_fields_valid'];prefix+=1
 else:
  assert not row['prefix_observer'] and row['planner_act_called']
  if handoff is None:
   handoff=i;assert acq.low_count>=5 and t-source['time_s']<=.5+1e-6 and source['points']>=100
 assert not acq.failed
np.testing.assert_array_equal(req,np.array([r['request'] for r in rows],np.float32))
shadow=[json.loads(s) for s in (D/'B0_shadow_rows.ndjson').read_text().splitlines()];assert shadow
# Rows compare baseline on current factual state only. Do not use candidate suffix as baseline physics.
result=dict(decision='PASS_FACTUAL_PREFIX_AND_HANDOFF' if handoff is not None else 'REJECT_NO_HANDOFF',requests_exact=len(req),prefix_requests_exact=prefix,handoff_frame=handoff,handoff_time_s=states[handoff]['time_s'] if handoff is not None else None,current_measured_five_low_required=True,current_source_fresh=True,no_reset_inside=True,shadow_rows=len(shadow),not_alternate_B0_physics=True,not_navigation_acceptance=True,not_original35=True,whole_research_complete=False)
(H/'front_view_handoff_boundary_verification_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
