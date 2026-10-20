"""Independent seed interface: pooled MODEL birth differs from class-model birth."""
import copy
import numpy as np
from tracking_tube_planner import TrackingTubeGrid

class PersistentCoreGrid(TrackingTubeGrid):
 def seed(self,packet,now):
  t=float(packet['time_s'])
  if not np.isfinite(t) or not np.isfinite(now) or abs(t-now)>1e-9:raise ValueError('current pool before payload')
  if self.birth is not None:raise ValueError('immutable pool')
  if not packet['model_hypothesis'] or packet['measured_geometry'] or packet['certified_free'] or not packet['static_persistence_prior']:raise ValueError('declared MODEL persistence only')
  birth=float(packet['source_model_birth_time_s'])
  if float(packet['hypothesis_birth_time_s'])!=t or not np.isfinite(birth) or birth>t or float(packet['source_seed_time_s'])!=birth:raise ValueError('separate pool and original class birth')
  xy=np.asarray(packet['possible_xy'])
  if xy.ndim!=2 or xy.shape[1]!=2 or not len(xy) or not np.isfinite(xy).all():raise ValueError('nonempty finite pool')
  self.birth=copy.deepcopy(packet)
