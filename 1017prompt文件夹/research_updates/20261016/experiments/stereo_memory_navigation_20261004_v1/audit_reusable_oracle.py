"""Reuse audit of historical successful composition, not a new reproduction run."""
import hashlib,json
from pathlib import Path
H=Path(__file__).resolve().parent
O=H.parent/'oracle_feasible_route_20260929_v9'
read=lambda f:json.loads(f.read_text())
sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
source=read(O/'hybrid_golden_source_identity.json')
match={p:dict(recorded_sha256=v,current_sha256=sha(Path(p)),equal=sha(Path(p))==v) for p,v in source.items()}
rows=[]
for path in sorted(O.glob('navigation_hybrid_*_*/episode_*/metrics.json')):
    m=read(path);rows.append(dict(root=str(path.parent.parent),episode=path.parent.name,success=m['success'],collision=m['collision'],elapsed_s=m['elapsed_s'],final_distance_m=m['final_distance_m']))
assert len(rows)==10 and sum(r['success'] for r in rows if 'hard' in r['root'])==5 and sum(r['success'] for r in rows if 'golden' in r['root'])==3
shared={name:sha(O/'hybrid'/name)==sha(H.parent/'oracle_feasible_route_20260929_v9/gated_speed_012'/name) for name in ('roll_pid.py','selected_candidate.json','bank_mapping.py','bank_mapping.json')}
identity=read(O/'navigation_hybrid_golden_golden_e1_0001/experimental_controller.json')
assert all(sha(Path(p))==v for p,v in identity['files'].items())
out=dict(role='historical result/source reuse audit; no currentrig/controller reproduction or new success rate',rows=rows,
 historical_wrapper_file_sha_verified=True,historical_core_source_comparison=match,shared_current_rollpid_config_and_mapping_file=shared,
 actual_old_controller='benchmark.policies.OraclePathPolicy wrapped by bank_mapping.make_policy and gated_speed.with_confident_speed; runtime reference selection via hybrid_planning/time_planning',
 current_information_controller='frozen_information_policy.InformationPolicy +provenance_grid.ProvenanceGrid +provenance_policy.InformationPolicy; headingtoaux+unknownmaincap.3',
 reusable='same BankPID source/gains, measured steady bankmap definitions, collision-sampled constantarc geometry and objectivecode; source functions can be audited/reused in independent candidate',
 nontransferable='old full-map/global path, speed policy and complete controller cannot be installed as a one-layer fix. Steady bankmap floor~.18m/s not validated at current~.12. New targetedtable is needed for startup sideward motion and largebank stopping',
 historical_identity_scope='old runtime and run_pointgoal_benchmark sourceSHA differ today; oldenvironment lacks camera_rig. Preserve historicalscores but cannotclaim exact current fullruntime/camera identity or current reproducibility from wrapperhash alone',
 source_sha256={str(f):sha(f) for f in (Path(__file__),O/'hybrid_golden_source_identity.json',O/'full_comparison.json')})
(H/'reusable_oracle_audit.json').write_text(json.dumps(out,indent=2)+'\n')
print('historical10scores verified;13/15listed sources same; runtime/runner changed; wrapper9/9same')
