"""Provide an explicitly historical MODEL; current installation is not exposure."""
import copy,re,math
import numpy as np

def installable_model(seed,now,*,prior_sequence,prior_sha256):
 t=float(now)
 if not math.isfinite(t) or t<0 or not isinstance(prior_sequence,str) or not prior_sequence or re.fullmatch(r'[0-9a-f]{64}',prior_sha256) is None:raise ValueError('finite current installation and prior identity')
 if not seed['model_hypothesis'] or seed['measured_geometry'] or seed['certified_free'] or not seed['static_persistence_prior']:raise ValueError('historical MODEL only; no current observation/free claim')
 for k,cols in [('possible_xy',2),('raw_world_voxel_centres',3)]:
  a=np.asarray(seed[k]);
  if a.ndim!=2 or a.shape[1]!=cols or not len(a) or not np.isfinite(a).all():raise ValueError('finite unchanged historical geometry')
 oldtimes={k:float(seed[k]) for k in ['time_s','hypothesis_birth_time_s','source_model_birth_time_s','source_seed_time_s']}
 if not all(math.isfinite(v) for v in oldtimes.values()):raise ValueError('original source timestamp namespace required')
 r=copy.deepcopy(seed);r.update(time_s=t,hypothesis_birth_time_s=t,source_model_birth_time_s=-1.,source_seed_time_s=-1.,source_consumer='PROVIDED_HISTORICAL_MODEL_CONTROL_DIAGNOSTIC',provided_model_provenance=dict(prior_sequence=prior_sequence,prior_sha256=prior_sha256,prior_source_clock=oldtimes,prior_source_model_birth_time_s=oldtimes['source_model_birth_time_s'],current_observation=False,current_installation_not_acquisition=True,source_local_time_minus1_is_prior_epoch=True,new_source_admission=False,scene_persistence_unproved=True))
 return r
