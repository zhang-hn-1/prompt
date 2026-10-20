from pathlib import Path
import json,hashlib,numpy as np
from partial_point_risk_grid import PartialPointRiskGrid
H=Path(__file__).resolve().parent;D=H/'native_fixed_appearance_probe_run_v1';M=H/'fixed_appearance_raw_point_memory_v1';O=H/'partial_point_risk_grid_preflight_v3';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
init=np.load(D/'estimated_poses/frame_000.npz');g=PartialPointRiskGrid(init['root_position_world_m'][:2]-20,10);count=0;inputs={}
for r in json.loads((M/'memory_shadow_rows.json').read_text()):
 i=r['frame'];f=M/'tentative_memory_snapshots'/f'frame_{i:03d}.json';p=D/'estimated_poses'/f'frame_{i:03d}.npz';inputs[str(f)]=sha(f);inputs[str(p)]=sha(p);memory=json.loads(f.read_text());pose=np.load(p);t=r['time_s'];g.set_source(dict(time_s=t,records=memory['records']));g.update_points(np.empty((0,3)),pose['root_position_world_m'],t);gold=np.load(O/'maps'/f'frame_{i:03d}.npz')
 for key in ('labels','codes','hazard_acquisition_time','safe_start_prior'):np.testing.assert_array_equal(getattr(g,key),gold[key])
 count+=1
result=dict(decision='PASS_COMPATIBILITY_ADAPTER_CURRENT_ENVELOPE_ONLY',epochs=count,mapped_fields=4*count,no_planner_rerun=True,old_preflight_grid_and_nominal_plan_retained=True,review=dict(C=0,I=1,M=0,important_resolved='record clocks must not exceed envelope; all metadata checked before world'),inputs=inputs,code_sha256=sha(H/'partial_point_risk_grid.py'),old_pre_review_code_sha256=sha(H/'partial_point_risk_grid_before_envelope_fix_v1.py'),not_online=True,not_navigation=True,not_fresh=True,whole_research_complete=False)
(H/'partial_point_risk_grid_adapter_verification_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='inputs'},indent=2))
