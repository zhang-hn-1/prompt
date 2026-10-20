import importlib.util
import numpy as np
assert importlib.util.find_spec('ownership') is not None, 'ownership consumer not implemented'
from ownership import qualify

def packet(q,r):
 return dict(query_pixel_id=np.array(q,dtype=int),partner_pixel_id=np.array(r,dtype=int),time_s=1.,frame=5)
assert qualify(packet([2,4,6],[1,3,5]),1.)['keep'].tolist()==[True]*3
assert qualify(packet([2,4,6],[1,1,5]),1.)['keep'].tolist()==[False,False,True]
assert qualify(packet([6,4,2],[5,1,1]),1.)['keep'].tolist()==[True,False,False]
assert qualify(packet([],[]),1.)['keep'].size==0
for p,t in [(packet([2,2],[1,1]),1.),(packet([2],[1]),.9),(packet([2],[-1]),1.)]:
 try: qualify(p,t)
 except ValueError: pass
 else: raise AssertionError('invalid input accepted')
p=packet([2,4],[1,1]);before=p['partner_pixel_id'].copy();v=qualify(p,1.)
assert np.array_equal(before,p['partner_pixel_id']) and not bool(v['certified_free'])
print('8 focused cases passed')
