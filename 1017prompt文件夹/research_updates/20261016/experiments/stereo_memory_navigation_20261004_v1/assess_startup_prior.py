"""Post-consumer actual-sweep evaluation, NOT a runtime motion plan."""
from pathlib import Path
import json,hashlib,datetime,numpy as np
from startup_prior import ImmutableStartupPrior
from point_support_shadow import swept_indices,support_result
H=Path(__file__).resolve().parent;out=H/'startup_prior_runs';p=json.loads((H/'startup_prior_protocol.json').read_text());m=json.loads((out/'manifest.json').read_text());sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
assert m['future_trajectory_not_read'] and len(m['rows'])==65
for r in m['rows']:
 for f,d in [('path','sha256'),('packet_path','packet_sha256'),('map_path','map_sha256')]:assert sha(r[f])==r[d]
prior_raw=np.load(out/'immutable_prior.npz');priorcells=prior_raw['cells'];prior=ImmutableStartupPrior(priorcells,float(prior_raw['time_s']),source='task_assumption',trusted=True)
states=json.loads((H/'dense_adapter_heldout_run/states.json').read_text());xy=np.array([s['position'][:2] for s in states])[60:];sweep=swept_indices(xy,p['origin'],.1,.7);assert len(sweep)==498;pset=set(map(tuple,priorcells));outside=np.array([c for c in sweep if tuple(c) not in pset]);assert len(priorcells)==185 and len(outside)==313
np.savez_compressed(out/'offline_actual_sweep.npz',xy=xy,cells=sweep,cells_outside_prior=outside,is_online_plan=False)
rows=[];points=0;perfect_floor=0;peakfloor=0
for mr in m['rows']:
 r=json.loads(Path(mr['path']).read_text());maps=np.load(mr['map_path']);a=maps['labels'];q=np.load(mr['packet_path']);points+=len(q['world']);perfect_floor+=int((np.abs(q['world'][:,2])<=.05).sum());peakfloor=max(peakfloor,r['floor_cells']);future=support_result(a,sweep);conditional=None if mr['frame']<60 else prior.evaluate(a,sweep,mr['time_s']);outer=support_result(a,outside)
 rows.append(dict(frame=mr['frame'],time_s=mr['time_s'],current_cold=r['cold'],current_prior=r['with_prior'],full_sweep_cold=future,full_sweep_prior=conditional,outside_prior=outer,floor_cells=r['floor_cells'],hazard_cells=r['hazard_cells']))
post=[r for r in rows if r['frame']>=60];startup=next(r for r in rows if r['frame']==60)
summary=dict(total_visible_endpoint_records=points,total_true_floor_records=perfect_floor,peak_floor_cells=peakfloor,whole_sweep_cells=len(sweep),immutable_prior_cells=len(priorcells),outside_prior_cells=len(outside),cold_disk_passes=sum(r['current_cold']['support_passed'] for r in rows),prior_disk_passes_after_activation=sum(r['current_prior']['necessary_support_passed'] for r in post),prior_post_frames=len(post),cold_whole_passes=sum(r['full_sweep_cold']['support_passed'] for r in rows),prior_whole_passes=sum(r['full_sweep_prior']['necessary_support_passed'] for r in post),minimum_outside_prior_unknown=min(r['outside_prior']['unknown_cells'] for r in post),startup=startup)
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),maps_manifest_utc=m['saved_utc'],summary=summary,rows=rows,privileged_diagnostic_only=True,prior_not_observation=True,task_prior_alone_solves_navigation=False,full3D_or_RGB_or_online_unvalidated=True,whole_research_complete=False)
(out/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in summary.items() if k!='startup'},indent=2));print('startup',json.dumps({a:{k:v for k,v in startup[a].items() if not k.endswith('indices')} for a in ['current_cold','current_prior','full_sweep_cold','full_sweep_prior','outside_prior']},indent=2))
