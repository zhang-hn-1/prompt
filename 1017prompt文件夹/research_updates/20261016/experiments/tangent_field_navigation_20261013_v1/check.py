import numpy as np
from field import target
assert np.allclose(target([.2,0],[0,0],[1,0],.5),[.7,0])
assert target([.2,.2],[0,0],[1,0],.5)[1]<.2
assert target([.2,-.2],[0,0],[1,0],.5)[1]>-.2
assert target([.8,0],[1,0],[0,0],.5)[0]<.8
assert np.isclose(np.linalg.norm(target([.2,.2],[0,0],[1,0],.5)-[.2,.2]),.5)
for bad in [[float('nan'),0],[0,0,0]]:
 try:target(bad,[0,0],[1,0],.5)
 except ValueError:pass
 else:raise AssertionError('invalid accepted')
try:target([0,0],[0,0],[0,0],.5)
except ValueError:pass
else:raise AssertionError('zero segment accepted')
print('8 geometric checks passed')
