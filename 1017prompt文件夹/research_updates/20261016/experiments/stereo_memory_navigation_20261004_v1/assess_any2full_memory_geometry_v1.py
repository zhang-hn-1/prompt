"""Labels read only after frozen predictions exist; ALL nonempty eyes."""
from pathlib import Path
import json,hashlib,datetime,numpy as np
from predicted_geometry_metrics import evaluate_geometry
H=Path(__file__).resolve().parent;I=H/'any2full_memory_inputs_v1';O=H/'any2full_memory_predictions_v1';D=H/'qualified_hazard_navigation_run_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();out=H/'any2full_memory_geometry_assessment_v1.json';assert not out.exists()
s=json.loads((O/'prediction_seal.json').read_text());assert all(sha(p)==v for p,v in s['output_sha256'].items());assert all(sha(p)==v for p,v in s['source_sha256'].items());assert all(sha(p)==v for p,v in json.loads((I/'input_seal.json').read_text())['input_sha256'].items());rows=[];gtpaths=[]
for r in json.loads((O/'records.json').read_text()):
 if not r['actual_learned_forward']:
  rows.append(dict(frame=r['frame'],eye=r['eye'],status=r['status'],gate=None,NOT_success=True));continue
 name=f"frame_{r['frame']:03d}_{r['eye']}.npz";f=np.load(O/name);i=np.load(I/name);gp=D/'offline_truth'/f"frame_{r['frame']:03d}.npz";mp=D/'masks'/f"frame_{r['frame']:03d}.npz";g=np.load(gp);m=np.load(mp);gtpaths.extend([gp,mp]);assert float(g['time_s'])==r['current_time_s'];assert np.array_equal(g[r['eye']+'_camera_usd_world'],i['current_camera_usd_world']);assert not bool(f['prediction_is_observation']);metrics=evaluate_geometry(f['depth_m'],g[r['eye']+'_depth_m'],i['current_camera_usd_world'],m[r['eye']],i['prompt_depth_native_m'],g['root_position_world_m']);rows.append(dict(frame=r['frame'],eye=r['eye'],status='POSTSEAL_DEVELOPMENT',**metrics))
active=[r for r in rows if r['gate'] is not None];passed=len(active)==3 and all(r['gate'] for r in active)
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),prediction_seal_sha256=sha(O/'prediction_seal.json'),assessment_source_sha256=sha(Path(__file__)),GT_postseal_sha256={str(p):sha(p) for p in gtpaths},rows=rows,decision='DEVELOPMENT_ONLY_PREDICTED_LAYER_GATE_PASS' if passed else 'REJECT_PREDICTED_GEOMETRY_LAYER',confidence_available=False,uncertainty_calibrated=False,knownpose_privileged=True,fresh_heldout=False,controller_admitted=False,formal_navigation_triggered=False,whole_research_complete=False)
out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
