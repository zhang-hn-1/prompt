import json,math
from pathlib import Path
import numpy as np
from source_map import SourceMapReplanner,ROOT
from benchmark.drive_model import DriveModel
from collection_schema import load_contract
from scripts.bev_ground_truth import crop_global_occupancy,obstacle_clearance
from probe import write
HERE=Path(__file__).resolve().parent
model=DriveModel.from_json(ROOT/'research/ball_wm/configs/benchmark_v1/drive_dynamics_v1.json');contract=load_contract(ROOT/'research/ball_wm/configs/data_contract.yaml')
results={}
for case in ['0002','0004']:
    adapter=SourceMapReplanner(case,model);results[case]={}
    for group in ['real','reserve']:
        p=HERE/f'{case}_{group}';z=np.load(p/'trace.npz');r=json.loads((p/'tracking.json').read_text());clearances=[]
        for xy,q in zip(z['position'],z['quaternion']):
            yaw=math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]**2+q[3]**2))+adapter.yaw
            pos=adapter.rotation@xy+adapter.origin
            bev=crop_global_occupancy(adapter.map.grid,adapter.map.global_origin_xy,np.array([*pos,.4]),yaw,contract.bev)
            clearances.append(obstacle_clearance(bev,.4,contract.bev))
        bad=np.flatnonzero(np.asarray(clearances)<=0.)
        commands=z['reference_command'][:-1];nominal=0.;capped=0.
        for main,aux in commands:
            k=model.curvature(main,aux);mag=abs(main)
            effective=min(mag,.5) if abs(k)>=.55 else min(mag,1.) if abs(k)>=.25 else mag
            target=model.auxiliary_for_curvature(math.copysign(effective,main),k)
            nominal+=.3/model.speed(main,aux)
            capped+=.3/model.speed(math.copysign(effective,main),target)
        normal=[x for x in r if x['source']=='tracking']
        bins={str(v):sum(math.isclose(abs(x['policy_protocol_main_command_rad_s']),v,abs_tol=1e-6) for x in normal) for v in [.5,1.,1.5]}
        results[case][group]=dict(first_offline_runtime_predicate_collision_s=None if not len(bad) else float(z['time'][bad[0]]-z['time'][0]),minimum_offline_runtime_clearance_m=min(clearances),reference_V1_nominal_motion_time_s=nominal,reference_V1_with_B0_curvature_caps_motion_time_s=capped,time_estimate_scope='quasistatic diagnostic; excludes startup, braking, dynamic deviation; not guaranteed bound',tracking_command_blocks_by_magnitude=bins,tracking_blocks=len(normal),reference_saturated_aux_primitives=sum(abs(a)>=math.pi/6-1e-7 for m,a in commands))
        np.savez(p/'offline_original_map_clearance.npz',time=z['time'],clearance_m=clearances)
write(HERE/'diagnosis.json',results)
print((HERE/'diagnosis.json').read_text())
