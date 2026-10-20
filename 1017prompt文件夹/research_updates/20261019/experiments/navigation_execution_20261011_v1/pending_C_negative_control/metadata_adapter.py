"""Exact view metadata used by sealed C replay; no source-point changes."""
import numpy as np
def add_view_metadata(records,body,local):
 body=np.asarray(body,float)
 if body.shape!=(4,4) or not np.isfinite(body).all():raise ValueError('finite causal estimatedbody required')
 out=[]
 for record in records:
  n=record['query_camera']
  if n not in ('front_left','front_right','rear_left','rear_right'):raise ValueError('unknown sourceeye')
  pair=(0,1) if n.startswith('front') else (2,3)
  centre=(local[pair[0]][:3,3]+local[pair[1]][:3,3])/2
  r=dict(record);r['body_center']=body[:3,3].copy();r['pair_center']=body[:3,3]+body[:3,:3]@centre
  out.append(r)
 return out
