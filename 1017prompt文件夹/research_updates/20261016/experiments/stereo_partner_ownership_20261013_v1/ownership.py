"""Reject every claimant of a repeated rounded right pixel in a current source packet.
Packet population is the existing CAD/range/stride2 selected source, not all image
pixels. Integer collisions are only ambiguity heuristics, not proof of error.
No GT, new depth, free-space labels, or history erasure.
"""
import numpy as np

def qualify(packet,now):
 t=float(packet['time_s']);q=np.asarray(packet['query_pixel_id']);r=np.asarray(packet['partner_pixel_id'])
 if not np.isfinite(t) or t!=float(now): raise ValueError('current causal clock')
 if q.ndim!=1 or r.shape!=q.shape: raise ValueError('correspondence shape')
 for ids in (q,r):
  if not np.issubdtype(ids.dtype,np.integer) or (ids<0).any() or (ids>=640*480).any(): raise ValueError('native pixel ids')
 if len(np.unique(q))!=len(q): raise ValueError('duplicate query identity')
 _,inverse,counts=np.unique(r,return_inverse=True,return_counts=True)
 keep=counts[inverse]==1
 return dict(query=q.copy(),partner=r.copy(),keep=keep,claimants=counts[inverse],time_s=t,frame=int(packet['frame']),certified_free=False)
