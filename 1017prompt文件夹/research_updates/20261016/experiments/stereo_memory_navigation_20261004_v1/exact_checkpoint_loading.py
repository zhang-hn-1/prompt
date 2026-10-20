"""Reject incomplete/mismatched checkpoint before mutating any model weight."""
import torch

def exact_load(model,state):
 expected=model.state_dict()
 if set(expected)!=set(state):raise ValueError('checkpoint key set mismatch; no fallback')
 for key,tensor in expected.items():
  incoming=state[key]
  if not isinstance(incoming,torch.Tensor) or incoming.shape!=tensor.shape or incoming.dtype!=tensor.dtype:raise ValueError('checkpoint shape/dtype mismatch: '+key)
  if incoming.is_floating_point() and not torch.isfinite(incoming).all():raise ValueError('nonfinite checkpoint tensor: '+key)
 model.load_state_dict(state,strict=True)
 for key,tensor in model.state_dict().items():
  if not torch.equal(tensor.detach().cpu(),state[key].detach().cpu()):raise ValueError('final checkpoint equality failed: '+key)
 return len(state)
