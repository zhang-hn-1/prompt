import importlib.util
assert importlib.util.find_spec('model') is not None,'model missing'
import torch,numpy as np
from model import VisibleMapNet,tentative
m=VisibleMapNet();x=torch.zeros(2,6,120,160);g=torch.tensor([[0.,0.,-1.]]*2);z=m(x,g);assert z.shape==(2,5,40,60)
a=torch.zeros(1,5,40,60);a[:,0]=1;labels,prob=tentative(a);assert not labels.any()
a.fill_(-10);a[:,2]=10;labels,prob=tentative(a);assert (labels==2).all()
a.fill_(0);labels,prob=tentative(a);assert not labels.any()
try:m(x,torch.zeros(2,4))
except ValueError:pass
else:raise AssertionError('gravity shape')
assert all('GT' not in k for k in m.state_dict())
print('5 model/consumer cases passed; ground never free')
