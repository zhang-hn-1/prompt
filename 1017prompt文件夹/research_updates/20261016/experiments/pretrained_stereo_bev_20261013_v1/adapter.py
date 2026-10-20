"""One unmodified official pretrained model; current RGB only, no truth."""
from pathlib import Path
import sys,os,ast,types,json,numpy as np,torch,cv2
D=Path(__file__).resolve().parent;V=D/'vendor/FoundationStereo'
os.environ['XFORMERS_DISABLED']='1'
sys.path.insert(0,str(V))
# Load exactly the two official pure utilities needed by model extractor.
# Avoid importing unused point-cloud/UI packages; function AST is unchanged.
tree=ast.parse((V/'Utils.py').read_text());nodes=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ['freeze_model','get_resize_keep_aspect_ratio']];assert len(nodes)==2
utility=types.ModuleType('Utils');utility.__dict__.update(np=np,torch=torch,cv2=cv2);exec(compile(ast.Module(body=nodes,type_ignores=[]),str(V/'Utils.py'),'exec'),utility.__dict__);sys.modules['Utils']=utility
from core.foundation_stereo import FoundationStereo
from core.utils.utils import InputPadder
from omegaconf import OmegaConf
from stereo_geometry import rectify_pair,restore_native,disparity_depth
class PretrainedStereo:
 def __init__(self,device):
  self.device=device;p=D/'weights/23-51-11/model_best_bp2.pth';assert p.exists(),'official weight download incomplete';cfg=OmegaConf.load(p.with_name('cfg.yaml'));cfg.vit_size='vitl';
  # Official documented local DINO source route; pretrained_dino remains False.
  hubload=torch.hub.load
  def local_hub(repo,name,*args,**kw):
   assert repo=='facebookresearch/dinov2' and not kw.get('pretrained',False)
   return hubload(str(V/'dinov2'),name,*args,source='local',**kw)
  torch.hub.load=local_hub
  import timm
  create=timm.create_model
  def no_initial_weights(*args,**kw):
   # Entire model checkpoint loads strict immediately; no initializer weights survive.
   kw['pretrained']=False;return create(*args,**kw)
  timm.create_model=no_initial_weights
  try:self.model=FoundationStereo(cfg)
  finally:torch.hub.load=hubload;timm.create_model=create
  ckpt=torch.load(p,map_location='cpu',weights_only=False);self.model.load_state_dict(ckpt['model'],strict=True);self.model.to(device).eval();torch.set_grad_enabled(False)
 def _disp(self,a,b):
  a=torch.as_tensor(np.ascontiguousarray(a),device=self.device).float().permute(2,0,1)[None];b=torch.as_tensor(np.ascontiguousarray(b),device=self.device).float().permute(2,0,1)[None];pad=InputPadder(a.shape,divis_by=32,force_square=False);a,b=pad.pad(a,b)
  with torch.inference_mode(),torch.autocast('cuda',enabled=True):d=self.model(a,b,iters=32,test_mode=True)
  return pad.unpad(d.float())[0,0].cpu().numpy()
 def predict(self,left,right,left_mask,right_mask):
  l,r=rectify_pair(left,right);lm,rm=rectify_pair(left_mask,right_mask);dl=self._disp(l,r);dr=-self._disp(np.flip(r,axis=1),np.flip(l,axis=1))[:,::-1].copy();z,valid=disparity_depth(dl,dr,lm,rm,lr_tolerance_px=1.5)
  return dict(depth_m=restore_native(z),confidence=restore_native(valid),disparity_px=dl)
