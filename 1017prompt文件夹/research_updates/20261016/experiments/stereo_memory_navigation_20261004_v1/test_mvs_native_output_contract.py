from pathlib import Path
import sys,types
import torch
from mvs_native_output_contract import configure_native_output

def test_official_decoder_native_resolution_and_checkpoint_weights_unchanged(monkeypatch):
 A=Path(__file__).resolve().parent/'mvsanywhere_source_preflight_v1'
 monkeypatch.syspath_prepend(str(A/'source/src'));monkeypatch.syspath_prepend(str(A/'isolated_packages'))
 from mvsanywhere.modules.depth_anything_blocks import DPTHead
 threads=torch.get_num_threads();torch.set_num_threads(1)
 try:
  head=DPTHead(model_name='dinov2_vitb14',prediction_scale=.875).eval();state={k:v.clone() for k,v in head.state_dict().items()};features=[(torch.zeros(1,6,768),torch.zeros(1,768)) for _ in range(4)]
  with torch.inference_mode():before=head(features,2,3)['log_depth_pred_s0_b1hw']
  assert tuple(before.shape[-2:])==(28,42)
  configure_native_output(types.SimpleNamespace(depth_decoder=head))
  with torch.inference_mode():after=head(features,2,3)['log_depth_pred_s0_b1hw']
  assert tuple(after.shape[-2:])==(32,48)
  assert all(torch.equal(v,state[k]) for k,v in head.state_dict().items())
 finally:torch.set_num_threads(threads)
