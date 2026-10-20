"""Current RGB only; independent pretrained metric depth and semantic category priors."""
from pathlib import Path
import sys,json,numpy as np,torch
from transformers import SegformerConfig,SegformerFeatureExtractor,SegformerForSemanticSegmentation
D=Path(__file__).resolve().parent;M=D.parent/'dual_monocular_bev_20261015_v1';sys.path.insert(0,str(M/'vendor/Depth-Anything-V2/metric_depth'))
from depth_anything_v2.dpt import DepthAnythingV2
class Model:
 def __init__(self):
  self.depth=DepthAnythingV2(encoder='vits',features=64,out_channels=[48,96,192,384],max_depth=20);self.depth.load_state_dict(torch.load(M/'weights/metric_hypersim_vits.pth',map_location='cpu',weights_only=True),strict=True);self.depth.cuda().eval()
  self.seg=SegformerForSemanticSegmentation(SegformerConfig.from_json_file(str(D/'semantic_weights/config.json')));self.seg.load_state_dict(torch.load(D/'semantic_weights/pytorch_model.bin',map_location='cpu',weights_only=True),strict=True);self.seg.cuda().eval();self.pre=SegformerFeatureExtractor.from_pretrained(str(D/'semantic_weights'),local_files_only=True)
 def eye(self,rgb):
  upright=np.ascontiguousarray(np.rot90(rgb));bgr=np.ascontiguousarray(upright[:,:,::-1])
  with torch.inference_mode():
   z=self.depth.infer_image(bgr,input_size=518);inp=self.pre(images=upright,return_tensors='pt');logits=self.seg(**{k:v.cuda() for k,v in inp.items()}).logits;logits=torch.nn.functional.interpolate(logits,size=upright.shape[:2],mode='bilinear',align_corners=False);prob,lab=logits.softmax(1).max(1);prob=prob[0].cpu().numpy();lab=lab[0].cpu().numpy()
  ground=np.isin(lab,[3,6,9,11,13,28,29,46,52,54,91,94]);unknown=np.isin(lab,[2,5,21,26,27,60]);category=np.where(prob>=.8,np.where(ground,2,np.where(unknown,0,1)),0).astype(np.uint8)
  return dict(depth_m=np.ascontiguousarray(np.rot90(z,-1)),category=np.ascontiguousarray(np.rot90(category,-1)),semantic_id=np.ascontiguousarray(np.rot90(lab.astype(np.uint8),-1)),semantic_confidence=np.ascontiguousarray(np.rot90(prob,-1)))
 def pair(self,left,right):return self.eye(left),self.eye(right)
