"""ONE Hero, SIX fixed causal cases. No GT/actions/world-state arrays read."""
from pathlib import Path
import sys,os,json,hashlib,datetime,time,copy,gc
H=Path(__file__).resolve().parent;A=H/'mvsanywhere_source_preflight_v1';O=H/'qualified_hazard_navigation_run_v1';D=H/'mvs_hero_temporal_development_v1';assert not D.exists();D.mkdir()
sys.path[:0]=[str(A/'isolated_packages'),str(A/'source/src'),str(A/'DINOv2_source')]
import numpy as np,torch,cv2,antialiased_cnns
from mvs_capture_causality import captured_request
from mvs_causal_packet_bridge import OPTICAL4
from exact_checkpoint_loading import exact_load
from strict_bilinear_support import unmasked_linear_support
from temporal_ray_sweep import project_depth,OPTICAL
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();art=json.loads((A/'artifact_identity_v1.json').read_text());weights=A/'weights/mvsanywhere_hero.ckpt';assert sha(weights)==art['Hero_sha256'];assert torch.cuda.is_available();torch.manual_seed(0);torch.set_num_threads(4)
# Restrict constructor-only dependencies. Every final tensor is strict checked.
def pinned_hub(repo,name,*args,**kwargs):
 if repo!='facebookresearch/dinov2' or name!='dinov2_vitb14':raise ValueError('unplanned dependency initializer')
 from dinov2.hub.backbones import dinov2_vitb14
 return dinov2_vitb14(pretrained=False)
orig_hub=torch.hub.load;torch.hub.load=pinned_hub;orig_resnet=antialiased_cnns.resnet18
antialiased_cnns.resnet18=lambda pretrained=True,**kw:orig_resnet(pretrained=False,**kw)
from mvsanywhere.experiment_modules.sr_depth_model import DepthModel
ckpt=torch.load(weights,map_location='cpu',mmap=True,weights_only=False);opts=copy.deepcopy(ckpt['hyper_parameters']['opts']);original_opts=repr(opts);opts.da_weights_path=None
model=DepthModel(opts);nstate=exact_load(model,ckpt['state_dict']);torch.hub.load=orig_hub;antialiased_cnns.resnet18=orig_resnet
parameter_sha=hashlib.sha256()
for k,v in model.state_dict().items():parameter_sha.update(k.encode());parameter_sha.update(v.detach().cpu().contiguous().numpy().tobytes())
del ckpt;gc.collect();model=model.eval().cuda()
(D/'runtime_identity_v1.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),torch=torch.__version__,gpu=torch.cuda.get_device_name(),Hero_sha256=art['Hero_sha256'],DINOv2_commit=art['DINOv2_commit'],strict_equal_state_tensors=nstate,final_state_sha256=parameter_sha.hexdigest(),checkpoint_opts_repr=original_opts,only_initializer_overrides=['pinned_DINOv2_pretrained_False','antialiased_resnet18_pretrained_False','da_weights_path_None'],full_parameters_verified_equal_before_inference=True,code_sha256=sha(__file__),bridge_sha256=sha(H/'mvs_causal_packet_bridge.py'),strict_mask_sha256=sha(H/'strict_bilinear_support.py'),loader_sha256=sha(H/'exact_checkpoint_loading.py'),plan_sha256=sha(H.parent.parent/'docs/superpowers/plans/2026-10-07-one-hero-temporal-depth-development.md'),knownpose_privileged=True,development_not_fresh=True,no_control_or_labels=True),indent=2)+'\n')
custody=json.loads((A/'whole_source_input_seal_v1.json').read_text());assert all(sha(p)==v for p,v in custody['code_sha256'].items());clocks={int(k):v for k,v in json.loads((A/'selected_clock_authority_v1.json').read_text()).items()};assert sha(A/'selected_clock_authority_v1.json')==custody['selected_clock_authority_sha256']
inputs={};rows=[];K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
def load(p):return np.load(p)
def tensor(v):return torch.as_tensor(v,dtype=torch.float32,device='cuda')
mean=tensor([.485,.456,.406])[:,None,None];std=tensor([.229,.224,.225])[:,None,None]
def rgb(v):return (tensor(v).permute(2,0,1)/255.-mean)/std
off=np.array([(x,y) for y in range(-3,4) for x in range(-3,4)])
for i in [235,245,255]:
 for eye in ['left','right']:
  r=captured_request(O,i,eye,clocks,load)
  for j in [i,i-10,i-20]:
   for kind in ['online_inputs','known_poses','masks']:
    p=O/kind/f'frame_{j:03d}.npz';digest=sha(p);assert digest==custody['input_sha256'][str(p.resolve())];inputs[str(p.resolve())]=digest
  cur=dict(time_s=r['current_time_s'],pose=r['cur_pose']@OPTICAL4);src=[dict(time_s=r['source_time_s'][j],pose=r['src_pose'][j]@OPTICAL4,mask=r['source_masks'][j]) for j in range(2)];k=tensor(r['cur_intrinsics'])[None];sk=tensor(r['src_intrinsics'])[None];k[:,:2]/=4;sk[:,:,:2]/=4;cp=tensor(r['cur_pose'])[None];sp=tensor(r['src_pose'])[None]
  cd=dict(image_b3hw=rgb(r['cur_image'])[None],world_T_cam_b44=cp,cam_T_world_b44=torch.linalg.inv(cp),K_matching_b44=k,invK_matching_b44=torch.linalg.inv(k),min_depth=tensor([.5]),max_depth=tensor([3.]))
  sd=dict(image_b3hw=torch.stack([rgb(v) for v in r['src_image']])[None],world_T_cam_b44=sp,cam_T_world_b44=torch.linalg.inv(sp),K_matching_b44=sk,invK_matching_b44=torch.linalg.inv(sk))
  torch.cuda.synchronize();torch.cuda.reset_peak_memory_stats();begin=time.perf_counter()
  with torch.inference_mode():out=model(phase='test',cur_data=cd,src_data=sd,return_mask=True,num_refinement_steps=1,estimate_depth_range=False)
  torch.cuda.synchronize();elapsed=time.perf_counter()-begin;depth=out['depth_pred_s0_b1hw'].squeeze().cpu().numpy();sky=out['sky_pred_s0_b1hw'].squeeze().cpu().numpy();peak=torch.cuda.max_memory_allocated();assert depth.shape==(480,640)
  yy,xx=np.indices(depth.shape);cvalid=cv2.boxFilter((~r['current_mask']).astype(np.float32),-1,(7,7),normalize=True,borderType=cv2.BORDER_CONSTANT)==1.;sel=np.isfinite(depth)&(depth>=.5)&(depth<=3.)&cvalid&(yy%2==0)&(xx%2==0);uv=np.c_[xx[sel],yy[sel]];z=depth[sel];rays=np.c_[uv,np.ones(len(uv))]@np.linalg.inv(K).T;T=cur['pose'];w=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@OPTICAL).T;Rb=(T[:3,:3]@OPTICAL)@np.array([[0,0,1],[0,-1,0],[1,0,0]]).T;root=T[:3,3]-Rb@np.array([.2,.3 if eye=='left' else -.3,.05]);haz=(w[:,2]>.05)&(w[:,2]<=1.5)&(np.linalg.norm(w-root,axis=1)>.55);uv=uv[haz];w=w[haz];z=z[haz];valid=np.ones(len(uv),bool)
  for start in range(0,len(uv),2048):
   stop=min(len(uv),start+2048);p=(uv[start:stop,None,:]+off).reshape(-1,2);pz=np.repeat(z[start:stop],49);good=np.ones(stop-start,bool)
   for s in src:
    xy,dz=project_depth(p,pz,T,s['pose'],K);mx=xy[:,0].reshape(-1,49).astype(np.float32);my=xy[:,1].reshape(-1,49).astype(np.float32);good&=(unmasked_linear_support(s['mask'],mx,my)&(dz.reshape(-1,49)>0)).all(1)
   valid[start:stop]=good
  before=len(uv);uv=uv[valid];w=w[valid];z=z[valid];past=np.array([project_depth(uv,z,T,s['pose'],K)[0] for s in src]);name=f'frame_{i:03d}_{eye}.npz'
  np.savez_compressed(D/name,depth_m=depth,sky_diagnostic_only=sky,world=w,current_uv=uv,past_uv=past,eye=eye,current_frame=i,source_frames=np.array([i-10,i-20]),query_pixel_id=uv[:,1]*640+uv[:,0],selected_depth_m=z,current_time_s=cur['time_s'],source_time_s=np.array([s['time_s'] for s in src]),geometry_available_time_s=cur['time_s'],derived_root_position_world_m=root,certified_free=False,source_model_admitted=False,learned_geometry_hypothesis_only=True)
  rows.append(dict(frame=i,eye=eye,time_s=cur['time_s'],file=name,latency_s=elapsed,peak_cuda_bytes=peak,pre_source_contract_hazards=before,hazard_queries=len(uv)));print(rows[-1],flush=True);del out,cd,sd;gc.collect()
(D/'records.json').write_text(json.dumps(rows,indent=2)+'\n');(D/'prediction_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256=inputs,output_sha256={str(p.resolve()):sha(p) for p in D.iterdir() if p.is_file()},whole_source_input_seal_sha256=sha(A/'whole_source_input_seal_v1.json'),artifact_identity_sha256=sha(A/'artifact_identity_v1.json'),source_identity_sha256=sha(A/'source_identity.json'),no_truth_or_actions_or_future_payload_input=True,all_six_predictions_before_assessment=True,development_not_fresh=True),indent=2)+'\n');print('SIX_HERO_SOURCE_PREDICTIONS_SEALED',flush=True)
