"""Unique nominal reference screen: continuous lines to unchanged occupied cell centers.
The original risk-cell half diagonal stays in the same planning margin. Unknown
remains unknown; this is neither completed volume nor certified dynamics/safety.
"""
import numpy as np
from execution_domain_four_point_consumer_v2 import planning_clearance

def continuous_cell_tube_screen(grid,guard):
 margin=planning_clearance(grid);lines=[]
 for key in ['bridge_xy','suffix_xy']:
  q=np.asarray(guard[key],float)
  if q.ndim!=2 or q.shape[1]!=2 or not len(q) or not np.isfinite(q).all():raise ValueError('finite current reference geometry')
  lines.append(q)
 endpoints=np.vstack(lines);cells=grid.cells(endpoints);shape=np.asarray(grid.labels.shape);inside=(cells>=0).all(axis=1)&(cells<shape).all(axis=1)
 result={'pass':False,'minimum_continuous_cell_clearance_m':None,'existing_planning_clearance_m':margin,'outside':int((~inside).sum()),'necessary_nominal_partial_screen_only':True,'no_free_or_completed_volume':True,'certified_safe':False,'risk_cell_geometry_unchanged':True,'path_quantization_removed_only':True}
 if not inside.all():return result
 # Same occupied exterior ring as original padded distance transform.
 x=np.arange(-1,shape[0]+1);y=np.arange(-1,shape[1]+1)
 border=np.vstack([np.column_stack([x,np.full(len(x),-1)]),np.column_stack([x,np.full(len(x),shape[1])]),np.column_stack([np.full(len(y),-1),y]),np.column_stack([np.full(len(y),shape[0]),y])])
 risks=np.vstack([np.argwhere(grid.labels==1),border]);world=grid.origin+risks*grid.resolution;minimum=np.inf
 for q in lines:
  if len(q)==1:minimum=min(minimum,float(np.linalg.norm(world-q[0],axis=1).min()))
  for a,b in zip(q[:-1],q[1:]):
   d=b-a;squared=float(np.dot(d,d));u=np.zeros(len(world)) if squared==0 else np.clip((world-a)@d/squared,0,1);minimum=min(minimum,float(np.linalg.norm(world-(a+u[:,None]*d),axis=1).min()))
 result.update(**{'pass':minimum>=margin-1e-8},minimum_continuous_cell_clearance_m=minimum,occupied_grid_cells=int((grid.labels==1).sum()),boundary_ring_preserved=True)
 return result
