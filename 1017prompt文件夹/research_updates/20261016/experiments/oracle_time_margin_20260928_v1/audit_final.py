import json,math,hashlib
from pathlib import Path
import numpy as np
from reserve_planner import sample_motion_primitive,execution_time_s,primitive_is_collision_free
from test_reserve import MODEL
from source_map import SourceMapReplanner
HERE=Path(__file__).resolve().parent
out={}
for case in ['0002','0004']:
    candidate=json.loads((HERE/f'{case}_plan.json').read_text());d=json.loads((HERE/f'{case}_diagnostic_no_time_cutoff.json').read_text());r=d['result'];info=dict(candidate=candidate['result'],diagnostic_status=r['status'],diagnostic_nodes=r['expanded_nodes'],formal_benchmark=False,physics_runs=0)
    assert candidate['candidate_sha256']==hashlib.sha256((HERE/'reserve_planner.py').read_bytes()).hexdigest()
    if r['status']=='success':
        adapter=SourceMapReplanner(case,MODEL);total=2.;xyerrors=[];yawerrors=[];clear=[]
        w=r['waypoints']
        for i,(a,b) in enumerate(zip(w,w[1:])):
            k=MODEL.curvature(a['main_command_rad_s'],a['auxiliary_angle_rad']);sgn=1 if a['drive_direction']=='forward' else -1
            samples=sample_motion_primitive(start_pose=(a['x_m'],a['y_m'],a['yaw_rad']),signed_arc_length_m=.3*sgn,curvature_1_m=k,sample_spacing_m=.05)
            residual=samples[-1]-[b['x_m'],b['y_m'],b['yaw_rad']];residual[2]=(residual[2]+math.pi)%(2*math.pi)-math.pi
            xyerrors.append(float(np.linalg.norm(residual[:2])));yawerrors.append(abs(float(residual[2])))
            world=samples.copy();world[:,:2]=samples[:,:2]@adapter.rotation.T+adapter.origin;world[:,2]+=adapter.yaw
            clear.append(primitive_is_collision_free(world,occupancy_grid=adapter.grid,origin_xy=adapter.map.global_origin_xy,resolution_m=adapter.map.resolution_m))
            total+=execution_time_s(a['main_command_rad_s'],a['auxiliary_angle_rad'],MODEL)
            if i and w[i-1]['drive_direction']!=a['drive_direction']:total+=3.8
        info.update(diagnostic_length_m=r['path_length_m'],search_time_s=r['estimated_time_s'],recomputed_time_s=total,max_xy_residual_m=max(xyerrors),max_yaw_residual_rad=max(yawerrors),all_primitives_clear_in_067_map=all(clear))
        assert max(xyerrors)<1e-8 and max(yawerrors)<1e-8 and all(clear)
        assert abs(total-r['estimated_time_s'])<1e-8
    out[case]=info
(HERE/'final_audit.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))
