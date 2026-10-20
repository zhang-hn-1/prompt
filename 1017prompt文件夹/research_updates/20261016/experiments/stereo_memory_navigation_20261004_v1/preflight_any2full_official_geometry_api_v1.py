"""Official preprocessing/alignment synthetic fixtures; no learned forward/load."""
from pathlib import Path
import sys,json,hashlib,datetime,importlib.util
from types import SimpleNamespace
import numpy as np,torch
H=Path(__file__).resolve().parent;S=H/'third_party/any2full_predicted_geometry_v1';A=H/'any2full_predicted_geometry_preflight_v1';sys.path.insert(0,str(S));from model.ours.any2full import Any2Full
torch.manual_seed(20261007);probe=Any2Full.__new__(Any2Full);torch.nn.Module.__init__(probe);rows=[]
for r in json.loads((H/'any2full_memory_inputs_v1/records.json').read_text()):
 f=np.load(H/'any2full_memory_inputs_v1'/f"frame_{r['frame']:03d}_{r['eye']}.npz")
 image=torch.from_numpy(f['current_rgb_world_up'].transpose(2,0,1).copy()).float()[None]/255.;depth=torch.from_numpy(f['prompt_depth_world_up_m'].copy())[None,None]
 ri,diff=probe.resize_to_multiple(image,mode='bicubic');rd,ddiff=probe.resize_to_multiple(depth,mode='nearest');assert ri.shape[-2:]==rd.shape[-2:] and diff==ddiff;back=probe.unresize(torch.full_like(rd,1.2345),ddiff);assert tuple(back.shape)==(1,1,640,480);torch.testing.assert_close(back,torch.full_like(back,1.2345));mean,std=probe.get_depth_bias_scale(probe.disparity_to_depth(rd));assert torch.isfinite(mean).all() and torch.isfinite(std).all();rows.append(dict(frame=r['frame'],eye=r['eye'],input_shape=list(depth.shape),official_internal_shape=list(rd.shape),original_nonzero_prompts=int((depth>0).sum()),official_nearest_nonzero_prompts=int((rd>0).sum()),inverse_depth_mean=float(mean[0]),inverse_depth_std=float(std[0]),model_should_execute=r['NN_should_execute'],actual_nn_forward_count=0))
pred=torch.linspace(.1,.9,32).reshape(1,1,4,8);truth_inverse=2*pred+3;result=probe.init_scailing(pred,truth_inverse);torch.testing.assert_close(result,truth_inverse,rtol=1e-5,atol=1e-5);empty=probe.disparity_to_depth(torch.zeros_like(pred));assert not empty.any()
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),official_resize_unresize_constant_axial_depth_PASS=True,official_positive_sparse_inverse_transform_PASS=True,official_current_prompt_affine_fixture_PASS=True,empty_prompt_skip_guard_from_our_inputs=True,checkpoint_loaded=False,learned_forward_calls=0,physics_calls=0,GT_input=False,torch_version=torch.__version__,hf_xet_installed=importlib.util.find_spec('hf_xet') is not None,hf_hub_installed=importlib.util.find_spec('huggingface_hub') is not None,rows=rows,code_sha256=sha(__file__),official_source_sha256=sha(S/'model/ours/any2full.py'))
p=A/'official_geometry_api_cpu_preflight_v1.json';assert not p.exists();p.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
