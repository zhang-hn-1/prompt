"""Pinned official inference modules, local verified weights; no shared installation."""
from pathlib import Path
import importlib,json,sys,types,hashlib
import numpy as np
import torch
from learned_correspondence import supported_features
H=Path(__file__).resolve().parent
def create_models():
    vendor=H/'third_party/lightglue_eb42fee';sys.path.insert(0,str(vendor/'dependency'))
    package=types.ModuleType('ballwm_frozen_lightglue');package.__path__=[str(vendor/'lightglue')];sys.modules[package.__name__]=package
    SuperPoint=importlib.import_module(package.__name__+'.superpoint').SuperPoint
    LightGlue=importlib.import_module(package.__name__+'.lightglue').LightGlue
    metadata=json.loads((H/'learned_feature_weights.json').read_text());allowed={v['url']:v for v in metadata['weights'].values()}
    def frozen_weights(url,*args,**kwargs):
        if url not in allowed:raise ValueError('unsealed weights URL')
        info=allowed[url];path=Path(info['path'])
        if hashlib.sha256(path.read_bytes()).hexdigest()!=info['sha256']:raise ValueError('weights mismatch')
        return torch.load(path,map_location='cpu',weights_only=True)
    previous=torch.hub.load_state_dict_from_url
    try:
        torch.hub.load_state_dict_from_url=frozen_weights
        extractor=SuperPoint(max_num_keypoints=2048).eval();matcher=LightGlue(features='superpoint').eval()
    finally:torch.hub.load_state_dict_from_url=previous
    return extractor,matcher

class FrozenLearnedFeatures:
    def __init__(self,device='cuda'):
        self.device=device;self.extractor,self.matcher=create_models();self.extractor.to(device);self.matcher.to(device)
    @torch.inference_mode()
    def extract(self,rgb,mask):
        image=torch.from_numpy(np.asarray(rgb).copy()).permute(2,0,1).float().to(self.device)/255.
        output=self.extractor.extract(image,resize=None)
        features={'xy':output['keypoints'][0].cpu().numpy(),'desc':output['descriptors'][0].cpu().numpy(),'scores':output['keypoint_scores'][0].cpu().numpy()}
        result=supported_features(features,mask);result['detected_before_mask']=len(features['xy']);return result
    @torch.inference_mode()
    def pairs(self,query,source):
        if not len(query['xy']) or not len(source['xy']):return np.empty((0,2),int)
        def tensors(view):
            return dict(keypoints=torch.from_numpy(view['xy'].copy()).float().to(self.device)[None],descriptors=torch.from_numpy(view['desc'].copy()).float().to(self.device)[None],image_size=torch.tensor([[640.,480.]],device=self.device))
        return self.matcher(dict(image0=tensors(query),image1=tensors(source)))['matches'][0].cpu().numpy()
