"""A possible range overlaps a known reference; no semantic obstacle or safety claim."""
import numpy as np

def _point(value):
    a=np.asarray(value,float)
    if a.shape!=(3,) or not np.isfinite(a).all():raise ValueError('finite XYZ point')
    return a

def _to_segment(p,a,b):
    d=b-a;n=d@d;t=0. if n<1e-20 else float(np.clip((p-a)@d/n,0.,1.))
    return float(np.linalg.norm(p-a-t*d))

def segment_distance(a,b,c,d):
    a,b,c,d=map(_point,(a,b,c,d));u=b-a;v=d-c;w=a-c
    distances=[_to_segment(a,c,d),_to_segment(b,c,d),_to_segment(c,a,b),_to_segment(d,a,b)]
    matrix=np.array([[u@u,-u@v],[-u@v,v@v]])
    if np.linalg.det(matrix)>1e-14:
        s,t=np.linalg.solve(matrix,[-u@w,v@w])
        if 0<=s<=1 and 0<=t<=1:distances.append(float(np.linalg.norm(w+s*u-t*v)))
    return min(distances)

def assess_reference(records,reference_xyz,now):
    points=np.asarray(reference_xyz,float);now=float(now)
    if points.ndim!=2 or points.shape[1]!=3 or not len(points) or not np.isfinite(points).all() or not np.isfinite(now):raise ValueError('finite current reference/time')
    segments=list(zip(points[:-1],points[1:])) or [(points[0],points[0])];rows=[];unusable=0
    for record in records:
        t=float(record['capture_time_s'])
        if not np.isfinite(t) or t>now:raise ValueError('future/nonfinite capture forbidden')
        if now-t>10.:continue
        if not record['world_ray_available']:unusable+=1;continue
        origin=_point(record['world_ray_origin'])
        for j,ray in enumerate(record['world_ray_directions']):
            ray=_point(ray);n=np.linalg.norm(ray)
            if n<1e-9:raise ValueError('nonzero world ray')
            ray=ray/n;a=origin+.5*ray;b=origin+3.*ray
            distance=min(segment_distance(a,b,c,d) for c,d in segments)
            rows.append(dict(capture_time_s=t,query_index=j,minimum_conditional_distance_m=distance,interval_intersects_body_proxy=bool(distance<=.4)))
    return dict(rows=rows,ambiguous_bearings=sum(r['interval_intersects_body_proxy'] for r in rows),tested_bearings=len(rows),pose_unusable_epochs=unusable,measured_obstacle=False,certified_free=False,control_candidate_admitted=False,role='conditional_reference_bearing_ambiguity',slant_range_prior_not_measurement=True)
