from pathlib import Path
import json,hashlib,datetime,numpy as np
H=Path(__file__).resolve().parent;sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
def main():
 p=json.loads((H/'tracking_capacity_protocol.json').read_text());a=json.loads((H/'tracking_capacity_assessment.json').read_text())
 for m in [p['source_sha256'],p['input_sha256']]:
  for f,d in m.items():assert sha(f)==d,f
 assert p['sealed_utc']<a['started_utc']<a['completed_utc'];assert [r['frame'] for r in a['rows']]==list(range(237,335))
 assert a['valid_tracking_denominator']==a['production_map_clamped_blocks']==a['protocol_clipped_blocks']==98
 assert len(a['windows'])==a['geometric_window_denominator']==a['geometric_available_windows']==10
 assert all(x['nominal_body_curvature_1_m']>x['current_bank_weighted_endpoint_support_1_m'][1] for x in a['rows'])
 assert [(x['position_frame'],x['map_frame'],x['guard']['reject_reasons']) for x in a['guard_counterfactuals']]==[(330,330,[]),(330,335,[]),(335,330,['original_raster_clearance']),(335,335,['original_raster_clearance'])]
 assert len(a['raster_witnesses'])==2
 for w in a['raster_witnesses']:assert w['source_frame']==131 and w['contributing_point_count']==3 and w['hazard_provenance_code']==4 and sha(H/'endpoint_navigation_run/sensor_packets/frame_131.npz')==w['source_packet_sha256']
 assert a['policy_act_calls']==a['planner_searches']==a['new_physical_sequences']==a['new_model_fits']==0 and a['control_candidate'] is None
 v=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_checks=len(p['source_sha256']),old_actual_input_checks=len(p['input_sha256']),valid_rows=98,available_windows=10,all_guards_source_and_limits_checks=True,no_science_replay=True,assessment_sha256=sha(H/'tracking_capacity_assessment.json'),old_navigation_gate_unchanged_rejected=True,whole_research_complete=False,automation='ACTIVE')
 (H/'tracking_capacity_verification.json').write_text(json.dumps(v,indent=2)+'\n');print(json.dumps(v,indent=2))
if __name__=='__main__':main()
