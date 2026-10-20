import torch,pytest
from exact_checkpoint_loading import exact_load

def test_complete_checkpoint_loads_every_parameter():
 m=torch.nn.Linear(2,1);s={'weight':torch.tensor([[3.,4.]]),'bias':torch.tensor([5.])};exact_load(m,s)
 assert torch.equal(m.weight,s['weight']) and torch.equal(m.bias,s['bias'])

def test_missing_checkpoint_key_rejected_before_mutation():
 m=torch.nn.Linear(2,1);old=m.weight.detach().clone()
 with pytest.raises(ValueError):exact_load(m,{'weight':torch.zeros_like(m.weight)})
 assert torch.equal(m.weight,old)

def test_shape_or_extra_key_rejected_before_mutation():
 m=torch.nn.Linear(2,1);old=m.weight.detach().clone()
 with pytest.raises(ValueError):exact_load(m,{'weight':torch.zeros(1,3),'bias':torch.zeros(1)})
 assert torch.equal(m.weight,old)
 with pytest.raises(ValueError):exact_load(m,{'weight':torch.zeros_like(m.weight),'bias':torch.zeros(1),'spare':torch.zeros(1)})
 assert torch.equal(m.weight,old)
