"""CPU selector only: at most12 nominal prefixes, unchanged guards; no live wrapper."""
import math,itertools
import numpy as np
from scipy.ndimage import distance_transform_edt
from benchmark.kinodynamic_planning import sample_motion_primitive
from reference_continuity_audit import reference_geometry_status
from route_geometry_audit import hazard_clearance

def choose_prefix(xy,yaw,goal_axis,grid,curvature_source):
 xy=np.asarray(xy,float);axis=np.asarray(goal_axis,float);labels=np.asarray(grid.labels);origin=np.asarray(grid.origin,float);resolution=float(grid.resolution)
 if xy.shape!=(2,) or axis.shape!=(2,) or origin.shape!=(2,) or not np.isfinite(np.r_[xy,axis,origin,yaw,resolution]).all() or np.linalg.norm(axis)==0 or labels.ndim!=2 or not np.isin(labels,[-1,0,1]).all() or abs(resolution-.1)>1e-9:raise ValueError('finite current frozen-grid/axis domain')
 axis=axis/np.linalg.norm(axis);margin=.6+.2+resolution*math.sqrt(2)/2
 field=distance_transform_edt(np.pad(labels!=1,1))[1:-1,1:-1]*resolution;hazards=origin+np.argwhere(labels==1)*resolution
 aux=(0.,-math.pi/6,math.pi/6);commands=[(.5,a) for a in aux];k={a:float(curvature_source(.5,a)) for a in aux}
 if not np.isfinite(list(k.values())).all():raise ValueError('finite frozen nominal curvature')
 root=np.r_[xy,float(yaw)]
 def step(p,a):return sample_motion_primitive(start_pose=tuple(p),signed_arc_length_m=.3,curvature_1_m=k[a],sample_spacing_m=.05)
 first={a:step(root,a) for a in aux};candidates=[]
 for a in aux:candidates.append(([commands[aux.index(a)]],np.vstack([root,first[a][-1]]),first[a]))
 for a,b in itertools.product(aux,repeat=2):
  tail=step(first[a][-1],b);candidates.append(([(.5,a),(.5,b)],np.vstack([root,first[a][-1],tail[-1]]),np.vstack([first[a],tail[1:]])))
 trace=[];best=None
 for requests,poses,samples in candidates:
  cells=np.floor((samples[:,:2]-origin)/resolution+.5).astype(int);inside=((cells>=0)&(cells<np.asarray(labels.shape))).all(1);raster=bool(inside.all() and (field[cells[:,0],cells[:,1]]>=margin).all()) if inside.all() else False
  continuous_clearance=hazard_clearance(samples[:,:2],hazards);continuous=continuous_clearance is None or continuous_clearance>=margin
  guard=reference_geometry_status(xy,poses[:,:2],labels,origin,resolution);progress=float((poses[-1,:2]-xy)@axis)
  eligible=bool(raster and continuous and guard['retention_necessary_screen_pass'] and progress>0.)
  row=dict(requests=requests,poses=poses.tolist(),samples_xy=samples[:,:2].tolist(),one_step=len(requests)==1,planning_clearance_m=margin,planning_raster_pass=raster,planning_continuous_pass=bool(continuous),guard=guard,goal_axis_progress_m=progress,selected_eligible=eligible,partial_reference=True,certified_safe=False,observed_free=False,old_local_target_reached=False)
  trace.append(row)
  if eligible and (best is None or progress>best['goal_axis_progress_m']):best=row
 return best,trace
