"""Independent evidence checks, including equality with rejected predecessor."""
import json,hashlib
from pathlib import Path
import numpy as np
from probe import write,ROOT
HERE=Path(__file__).resolve().parent
OLD=ROOT/'research/ball_wm/runs/benchmark_oracle_entry_recovery_20260928_v1'
results={}
for case in ['0002','0004']:
    p=HERE/f'{case}_candidate_source_map';r=json.loads((p/'replan.json').read_text());state=r['measured_start']
    records=json.loads((p/'tracking.json').read_text());e=json.loads((p/'status.json').read_text())['event'];post=records[e:e+80]
    brake=[i for i,x in enumerate(post) if x['source']=='direction_change_braking']
    initial_brake=0
    for x in post:
        if x['source']!='direction_change_braking':break
        initial_brake+=1
    current=np.load(p/'trace.npz');old=np.load(OLD/f'{case}_candidate_source_map/trace.npz')
    oldphysics=np.load(OLD/f'{case}_candidate_source_map/physics.npz');newphysics=np.load(p/'physics.npz')
    directions=[w['drive_direction'] for w in r['result']['waypoints']]
    results[case]=dict(measured_start=state,estimated_stop_plus_confirmation_s=(state['direction_change_cost_m']-.5)/state['speed_m_s'],initial_braking_request_s=.1*initial_brake,total_braking_request_s=.1*len(brake),initial_requested_direction=directions[0],geometric_direction_changes=r['result']['direction_changes'],direction_changes_including_initial=int(directions[0]!=state['initial_direction'])+r['result']['direction_changes'],equals_previous_candidate={k:bool(np.array_equal(current[k],old[k])) for k in ['position','proprio','quaternion','action']},all_physics_arrays_equal_to_previous_candidate=oldphysics.files==newphysics.files and all(np.array_equal(oldphysics[k],newphysics[k]) for k in oldphysics.files))
write(HERE/'supplement.json',results)
paths=['benchmark/policies.py','benchmark/kinodynamic_planning.py','benchmark/isaac_runtime.py','scripts/action_smoothing.py','configs/benchmark_v1/drive_dynamics_v1.json','configs/data_contract.yaml']
sources=HERE/'source_snapshot';sources.mkdir(exist_ok=True)
identities={}
for rel in paths:
    raw=(ROOT/'research/ball_wm'/rel).read_bytes();out=sources/rel;out.parent.mkdir(parents=True,exist_ok=True);out.write_bytes(raw);identities[rel]=hashlib.sha256(raw).hexdigest()
write(HERE/'source_identity.json',identities)
print(json.dumps(results,indent=2))
