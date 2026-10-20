from pathlib import Path
import json,hashlib,numpy as np
from tentative_point_memory import TentativePointMemory
from static_sparse_point_risk_grid_v1 import StaticSparsePointRiskGrid
from policy_row_journal_v1 import PolicyRowJournal,read_journal
H=Path(__file__).resolve().parent;D=H/'original_E1_direct_memory_navigation_run_v1';O=H/'static_sparse_memory_journal_CPU_v1';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((D/'artifact_manifest_post_close.json').read_text())
for p,d in seal['artifact_sha256'].items():assert sha(p)==d,p
rows=json.loads((D/'policy_records.json').read_text());sources=json.loads((D/'source_rows.json').read_text());states=json.loads((D/'states.json').read_text());expected=json.loads((H/'original_E1_memory_expiry_attribution_v1/cell_time_rows.json').read_text())
m=TentativePointMemory();g=StaticSparsePointRiskGrid(np.array([-3.2999999940395353,5.960465010446114e-9])-20,10,static_scene_assumption=True);cursor=0;clock=np.full(g.labels.shape,-np.inf);j=PolicyRowJournal(O/'rows.ndjson')
for i,r in enumerate(rows):
 t=r['time_s']
 while cursor<len(sources) and sources[cursor]['frame']<=i:
  s=sources[cursor];p=np.load(D/'partial_observations'/f"frame_{s['frame']:03d}.npz");cells=g.cells(p['world'][:,:2]);cells=cells[g.inside(cells)]
  if len(cells):np.maximum.at(clock,tuple(cells.T),s['time_s'])
  m.update(dict(frame=s['frame'],time_s=s['time_s'],world=p['world'],query_pixel_id=p['query_pixel_id'],partner_pixel_id=p['partner_pixel_id'],input_sha256=s['provenance']),s['time_s']);cursor+=1
 src=sources[cursor-1];g.set_source(dict(frame=src['frame'],time_s=src['time_s'],records=m.at(t)['records']));g.update_points(np.empty((0,3)),np.asarray(states[i]['position']),t)
 np.testing.assert_array_equal(g.hazard_acquisition_time,clock);np.testing.assert_array_equal(g.labels,np.where(np.isfinite(clock),1,-1))
 assert (g.labels==1).sum()==expected[i]['cumulative_past_observed_cells'];assert not np.any(g.labels==0) and not np.isfinite(g.memory.floor_time).any()
 planned={k:v for k,v in r.items() if k not in {'end_to_end_wall_s','execute_capture_NN_wall_s','wall_includes_policy_execute_render_NN_and_capture'}};j.planned(planned);j.executed(r)
j.close();reconstructed=read_journal(O/'rows.ndjson');assert reconstructed['completed']==rows and not reconstructed['pending']
a=dict(decision='PASS_CPU_ARCHIVED_CURRENT_PAST_ONLY_IMPLEMENTATION_AND_EXACT_JOURNAL',actual_new_physics=False,old_sequences_development_not_fresh=True,rows_exact=len(rows),static_cumulative_clocks_exact=len(rows),raw_record_TTL10_preserved=True,no_free_completed_volume_or_birth_refresh=True,not_navigation_admission=True,old_B0_complete60=False)
(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps(a,indent=2))
