"""Causal packet-prefix reconstruction for independent decision validation."""
import numpy as np
from spatial_memory import SpatialMemory

def replay_memory(packet_sources,last_current_index,loader):
    if last_current_index<0 or len(packet_sources)<=last_current_index:raise ValueError('missing current/past packet')
    memory=None
    for source in packet_sources[:last_current_index+1]:
        p=loader(source)
        if memory is None:memory=SpatialMemory(np.asarray(p['root_xyz'])[:2]-20,400,.1,max_age_s=10.)
        memory.update(p['points_world'],float(p['time_s']),pose_sigma_m=0.)
    return memory
