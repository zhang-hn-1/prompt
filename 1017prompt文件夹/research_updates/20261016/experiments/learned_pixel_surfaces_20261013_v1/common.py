from pathlib import Path
import sys,json,hashlib
D=Path(__file__).resolve().parent;W=D.parents[1];Q=W/'experiments/pixel_surface_projection_20261013_v1'
sys.path.insert(0,str(Q))
H=W/'experiments/stereo_memory_navigation_20261004_v1';C=W/'experiments/source_confidence_calibration_20261012_v1';O=W/'experiments/observed_end_subgoal_20261013_v1'
GROUPS={'H':W/'experiments/risk_height_navigation_20261012_v1/candidate_e2_run','E0':C/'data_e0_run','E1':C/'data_e1_run','B54':O/'baseline_e1_run','C54':O/'candidate_e1_run'}
sys.path[:0]=[str(W/'experiments/viewpoint_confirmation_navigation_20261012_v1'),str(H)]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(name,obj):
 (D/name).write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n')
def verify_hashes(mapping):
 for p,s in mapping.items():
  if sha(p)!=s:raise ValueError('sealed identity changed: '+p)
def verify_oracle():
 q=json.loads((Q/'qualification.json').read_text());assert q['qualification_pass']
 for key in ['input_sha256','output_sha256','source_sha256']:verify_hashes(q[key])
 assert json.loads((Q/'independent_closure.json').read_text())['offline_oracle_review_pass']
