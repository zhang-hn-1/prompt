"""One frozen completion prior; old prompts/current RGB only, never truth."""
from pathlib import Path
import sys,os,json,hashlib,datetime,time,zipfile,pickletools
from types import SimpleNamespace
import numpy as np,torch
H=Path(__file__).resolve().parent;A=H/'any2full_predicted_geometry_preflight_v1';I=H/'any2full_memory_inputs_v1';O=H/'any2full_memory_predictions_v1';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();p=json.loads((H/'any2full_memory_protocol_v2.json').read_text());assert all(sha(f)==v for f,v in p['source_sha256'].items());assert all(sha(f)==v for f,v in json.loads((I/'input_seal.json').read_text())['input_sha256'].items());assert (H/'any2full_memory_author_review_closed_v1.json').exists()
checkpoint=A/'Any2Full_vitl.pth.tar';assert checkpoint.stat().st_size==p['weight_size_bytes'] and sha(checkpoint)==p['weight_sha256']
with zipfile.ZipFile(checkpoint) as archive:
 names=[n for n in archive.namelist() if n.endswith('/data.pkl')];assert len(names)==1;data=archive.read(names[0]);assert len(data)<100000000;ops=list(pickletools.genops(data));assert not any(op.name=='STACK_GLOBAL' for op,arg,pos in ops),'unresolved stack GLOBAL format';globals_=sorted(set(arg for op,arg,pos in ops if op.name=='GLOBAL'))
(O/'checkpoint_static_globals.json').write_text(json.dumps(dict(sha256=sha(checkpoint),globals=globals_,weights_only_required=True),indent=2)+'\n')
# No execution of the official CLI loader (it uses weights_only=False).
loaded=torch.load(checkpoint,map_location='cpu',weights_only=True);state=loaded.get('state_dict',loaded);state={k.replace('module.',''):v for k,v in state.items()};assert state and all(isinstance(k,str) and isinstance(v,torch.Tensor) for k,v in state.items())
sys.path.insert(0,str(H/'third_party/any2full_predicted_geometry_v1'));from model.ours.any2full import Any2Full
args=SimpleNamespace(init_scailing=True,stage=1,min_depth=1e-6,max_depth=1000.)
model=Any2Full(encoder='vitl',da_ckpt_path=None,args=args);model.load_state_dict(state,strict=True);own=model.state_dict();assert set(own)==set(state)
for k in own:assert own[k].dtype==state[k].dtype and torch.equal(own[k],state[k]),k

def tensor_digest(values):
 digest=hashlib.sha256()
 for k,v in sorted(values.items()):
  x=v.detach().cpu().contiguous();digest.update(k.encode());digest.update(str(x.dtype).encode());digest.update(json.dumps(list(x.shape)).encode());digest.update(x.numpy().tobytes())
 return digest.hexdigest()
initial=tensor_digest(state);(O/'pre_forward_weight_identity.json').write_text(json.dumps(dict(state_entries=len(state),tensor_sha256=initial,strict_all_values_equal=True,da_init_loaded=False,weights_only=True),indent=2)+'\n');del own,state,loaded
assert torch.cuda.is_available();torch.manual_seed(20261007);model=model.to('cuda').eval();torch.cuda.reset_peak_memory_stats();rows=[];alignment=[];original=model.init_scailing

def current_prompt_alignment(pred,sparse,*args,**kwargs):
 idx=sparse.flatten()>1e-5
 snapshot=dict(pred_inverse_at_prompt=pred.flatten()[idx].detach().cpu().numpy().copy(),current_input_prompt_inverse=sparse.flatten()[idx].detach().cpu().numpy().copy())
 result=original(pred,sparse,*args,**kwargs)
 snapshot['official_aligned_inverse_at_prompt']=result.flatten()[idx].detach().cpu().numpy().copy();alignment.append(snapshot)
 return result
model.init_scailing=current_prompt_alignment
for r in json.loads((I/'records.json').read_text()):
 begin=time.perf_counter();f=np.load(I/f"frame_{r['frame']:03d}_{r['eye']}.npz");assert float(f['current_time_s'])==r['current_time_s'] and float(f['geometry_time_s'])<=r['current_time_s'] and r['predicted_is_observation'] is False
 if not r['NN_should_execute']:
  rows.append(dict(r,status='NOT_EXERCISED_EMPTY_PROMPT',actual_learned_forward=False));continue
 image=torch.from_numpy(f['current_rgb_world_up'].transpose(2,0,1).copy()).float()[None].cuda()/255.;image=(image-torch.tensor([.485,.456,.406],device='cuda')[None,:,None,None])/torch.tensor([.229,.224,.225],device='cuda')[None,:,None,None];depth=torch.from_numpy(f['prompt_depth_world_up_m'].copy())[None,None].cuda();assert int((depth>0).sum())==r['nonzero_prompts'];assert tuple(image.shape)==(1,3,640,480)
 torch.cuda.synchronize();start=time.perf_counter()
 with torch.no_grad():output=model(dict(rgb=image,dep=depth))
 torch.cuda.synchronize();elapsed=time.perf_counter()-start
 native=np.rot90(output['pred'][0,0].cpu().numpy(),-1).copy();raw=np.rot90(output['disparity_pre'][0,0].cpu().numpy(),-1).copy();assert native.shape==(480,640) and np.isfinite(native).all()
 name=f"frame_{r['frame']:03d}_{r['eye']}.npz";np.savez_compressed(O/name,depth_m=native,raw_pre_alignment_inverse=raw,current_time_s=r['current_time_s'],geometry_time_s=r['geometry_time_s'],eye=r['eye'],frame=r['frame'],source_frame=r['source_frame'],prediction_is_observation=False,uncertainty_calibrated=False,confidence_available=False,learned_prior=True)
 np.savez_compressed(O/name.replace('.npz','_actual_prompt_alignment.npz'),**alignment[-1],current_time_s=r['current_time_s'],geometry_time_s=r['geometry_time_s'],future_truth_scale_fit=False)
 rows.append(dict(r,status='PREDICTED_GEOMETRY_NOT_OBSERVATION',actual_learned_forward=True,official_current_prompt_inverse_alignment=True,forward_wall_s=elapsed,input_and_output_wall_s=time.perf_counter()-begin));print('PREDICTED',r['frame'],r['eye'],elapsed,flush=True)
final=tensor_digest(model.state_dict());assert initial==final;(O/'post_forward_weight_identity.json').write_text(json.dumps(dict(initial_tensor_sha256=initial,final_tensor_sha256=final,all_parameters_and_buffers_unchanged=True,peak_gpu_allocated_bytes=torch.cuda.max_memory_allocated(),GT_read=False),indent=2)+'\n');(O/'records.json').write_text(json.dumps(rows,indent=2)+'\n');(O/'prediction_seal.json').write_text(json.dumps(dict(utc=now(),output_sha256={str(f):sha(f) for f in O.iterdir() if f.is_file()},source_sha256=p['source_sha256'],actual_learned_forward_calls=sum(r['actual_learned_forward'] for r in rows),GT_read=False,development_not_fresh=True,whole_research_complete=False,predictions_not_observed_or_navigation_admission=True),indent=2)+'\n')
