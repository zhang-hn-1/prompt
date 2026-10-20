"""Independent nominal-reference measurement; no control or future feedback."""
import numpy as np
from scipy.ndimage import distance_transform_edt
from route_geometry_audit import hazard_clearance,dense_prefix


def reference_geometry_status(xy,path,labels,origin,resolution=.1,*,segment_interval=None):
    xy=np.asarray(xy,float);path=np.asarray(path,float);labels=np.asarray(labels);origin=np.asarray(origin,float)
    if xy.shape!=(2,) or origin.shape!=(2,) or path.ndim!=2 or path.shape[1]!=2 or len(path)<2 or labels.ndim!=2 or not np.isfinite(xy).all() or not np.isfinite(path).all() or not np.isfinite(origin).all() or not np.isfinite(resolution) or resolution<=0 or not np.isin(labels,[-1,0,1]).all():
        raise ValueError('finite old reference and observed map required')
    delta=np.diff(path,axis=0);length=np.linalg.norm(delta,axis=1)
    if np.any(length<=0):raise ValueError('duplicate reference vertices')
    arc=np.r_[0.,np.cumsum(length)]
    u=np.clip(((xy-path[:-1])*delta).sum(1)/length**2,0,1)
    candidates=path[:-1]+u[:,None]*delta;dist=np.linalg.norm(candidates-xy,axis=1)
    if segment_interval is None:
        j=int(dist.argmin())
    else:
        first,last=segment_interval
        if not isinstance(first,(int,np.integer)) or not isinstance(last,(int,np.integer)) or not 0<=first<=last<len(delta):
            raise ValueError('valid current drive leg required')
        j=int(first+dist[first:last+1].argmin())
    projection=candidates[j]
    projected=float(arc[j]+u[j]*length[j])
    suffix=np.vstack([projection,path[j+1:]])
    # At a vertex/end, retain a finite one-point suffix instead of duplicate vertices.
    suffix=suffix[np.r_[True,np.linalg.norm(np.diff(suffix,axis=0),axis=1)>1e-12]]
    bridge=np.vstack([xy,projection]);bridge=bridge[np.r_[True,np.linalg.norm(np.diff(bridge,axis=0),axis=1)>1e-12]]
    hazards=origin+np.argwhere(labels==1)*resolution
    suffix_clear=hazard_clearance(suffix,hazards);bridge_clear=hazard_clearance(bridge,hazards)
    continuous=all(x is None or x>=.6-1e-8 for x in [suffix_clear,bridge_clear])
    # Original planner checks rounded lattice positions against the padded EDT rule.
    field=distance_transform_edt(np.pad(labels!=1,1))[1:-1,1:-1]*resolution
    samples=np.vstack([dense_prefix(line,length_m=float(np.linalg.norm(np.diff(line,axis=0),axis=1).sum()),spacing_m=.05) for line in [suffix,bridge]])
    cells=np.rint((samples-origin)/resolution).astype(int)
    inside=((cells>=0)&(cells<np.asarray(labels.shape))).all(1)
    values=np.full(len(cells),-np.inf);values[inside]=field[cells[inside,0],cells[inside,1]]
    reasons=[]
    if not continuous:reasons.append('continuous_observed_hazard_clearance')
    if not bool((values>=.6-1e-8).all()):reasons.append('original_raster_clearance')
    if dist[j]>.5:reasons.append('projection_distance_over_0p5')
    if arc[-1]-projected<.5:reasons.append('remaining_arc_under_lookahead')
    return dict(projection_segment_index=j,projection_segment_interval=segment_interval,projected_arc_m=projected,projection_distance_m=float(dist[j]),remaining_arc_m=float(arc[-1]-projected),suffix_xy=suffix.tolist(),bridge_xy=bridge.tolist(),suffix_clearance_m=suffix_clear,bridge_clearance_m=bridge_clear,continuous_clearance_pass=continuous,raster_clearance_pass=bool((values>=.6-1e-8).all()),minimum_raster_clearance_m=float(values.min()) if inside.all() else None,raster_sample_count=len(cells),raster_outside_count=int((~inside).sum()),observed_hazard_count=len(hazards),retention_necessary_screen_pass=not reasons,reject_reasons=reasons,nominal_geometry_only=True,certified_safe=False)


def leg_interval(tracker):
    ref=tracker._reference;i=int(tracker._path_index)
    if tracker._pending_drive_direction is not None or tracker._braking_state!='tracking' or not 0<=i<len(ref)-1:
        return None
    direction=ref[i].drive_direction
    if direction not in ('forward','reverse') or tracker._active_drive_direction!=direction:return None
    a=b=i
    while a>0 and ref[a-1].drive_direction==direction:a-=1
    while b<len(ref)-2 and ref[b+1].drive_direction==direction:b+=1
    return a,b
