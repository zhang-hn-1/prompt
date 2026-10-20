from pathlib import Path
import sys
H=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(H))
p=H/'verify_RGB_memory_pose_causality_v1.py'
s=p.read_text().replace("out = D / 'causal_prefix_verification_v1.json'", "out = H / 'pose_stress_independent_review_checks_v1' / 'causal_prefix_personal.json'")
assert s!=p.read_text()
exec(compile(s,str(p),'exec'),{'__file__':str(p),'__name__':'__main__'})
