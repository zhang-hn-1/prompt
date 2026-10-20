import bootstrap
from dataclasses import asdict
from benchmark.kinodynamic_planning import KinodynamicWaypoint as WP,KinodynamicPlanResult as PR
from global_guidance import prefix_plan
p=PR('success',tuple(WP(i*.3,0,0,'forward' if i<5 else 'reverse',1 if i<5 else -1,0) for i in range(13)),17,3.6,1)
z=prefix_plan(p);assert len(z.waypoints)==9 and z.path_length_m==2.4 and z.direction_changes==1
assert z.waypoints[-1].main_command_rad_s==0 and z.waypoints[-1].drive_direction=='reverse'
assert all(asdict(a)==asdict(b) for a,b in zip(z.waypoints[:-1],p.waypoints))
s=PR('success',p.waypoints[:4],17,.9,0);q=prefix_plan(s);assert len(q.waypoints)==4 and abs(q.path_length_m-.9)<1e-9
print('prefix preserves first eight primitives, cusp identity, short tail and terminal zero')
