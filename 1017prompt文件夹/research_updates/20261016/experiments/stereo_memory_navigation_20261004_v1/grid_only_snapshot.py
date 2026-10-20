"""Read-only exact grid fields; raw provenance records remain in the mapper."""
import numpy as np
from point_support_shadow import finite_scalar

def grid_arrays_snapshot(mapper,time_s):
 t=finite_scalar(time_s)
 if t<mapper.time:raise ValueError('past query would read future evidence')
 floor=np.full(mapper.shape,-np.inf);hazard=np.full(mapper.shape,-np.inf);ignored=0
 # Read only retained records, preserving original query times and TTL.
 records=[r for r in mapper.records.values() if t-r['query'][3]<=mapper.age]
 if records:
  xyz=np.asarray([r['world'] for r in records],float);times=np.asarray([r['query'][3] for r in records],float);sigma=np.asarray([r['sigma_m']+r['pose_sigma_m'] for r in records],float)
  cell=np.rint((xyz[:,:2]-mapper.origin)/mapper.resolution).astype(int);inside=(cell>=0).all(1)&(cell[:,0]<mapper.shape[0])&(cell[:,1]<mapper.shape[1]);z=xyz[:,2]
  isfloor=inside&(np.abs(z)+sigma<=.05)&(sigma<=mapper.resolution*.5);ishazard=inside&(z>.05)&(z<=1.5)
  ignored=int((~(isfloor|ishazard)).sum())
  np.maximum.at(floor,(cell[isfloor,0],cell[isfloor,1]),times[isfloor]);np.maximum.at(hazard,(cell[ishazard,0],cell[ishazard,1]),times[ishazard])
 f=np.isfinite(floor);h=np.isfinite(hazard);labels=np.full(mapper.shape,-1,np.int8);labels[f]=0;labels[h]=1;codes=np.zeros(mapper.shape,np.uint8);codes[f]=np.where(np.abs(floor[f]-t)<1e-6,1,2);codes[h]=np.where(np.abs(hazard[h]-t)<1e-6,3,4)
 return dict(time_s=t,labels=labels,provenance=codes,floor_time=floor,hazard_time=hazard,ignored_records=ignored,conflict_cells=int((f&h).sum()),certified_free=False,source_model_admitted=False,full_3d_certified=False,model_imputed_regions=0)
