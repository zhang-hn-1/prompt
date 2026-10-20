from pathlib import Path
import sys,hashlib,json
D=Path(__file__).resolve().parent;W=D.parents[1];H=W/'experiments/stereo_memory_navigation_20261004_v1';V=W/'experiments/viewpoint_confirmation_navigation_20261012_v1';C=W/'experiments/source_confidence_calibration_20261012_v1';O=W/'experiments/observed_end_subgoal_20261013_v1';P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
sys.path[:0]=[str(V),str(H),str(W/'experiments/risk_height_navigation_20261012_v1'),str(P)]
GROUPS={'H':(W/'experiments/risk_height_navigation_20261012_v1/candidate_e2_run',W/'experiments/risk_height_navigation_20261012_v1/candidate_e2_protocol.json'),'E0':(C/'data_e0_run',C/'data_e0_protocol.json'),'E1':(C/'data_e1_run',C/'data_e1_protocol.json'),'B54':(O/'baseline_e1_run',O/'baseline_e1_protocol.json'),'C54':(O/'candidate_e1_run',O/'candidate_e1_protocol.json')}
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(name,value):
 p=D/name;p.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')

Q=W/"experiments/replanning_reserve_20261013_v1"
GROUPS.update({"B65":(Q/"baseline_e1_run",Q/"baseline_e1_protocol.json"),"C65":(Q/"candidate_e1_run",Q/"candidate_e1_protocol.json")})
