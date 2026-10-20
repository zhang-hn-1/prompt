"""Frozen probabilities; independent immutable RGB-only calibrated classification."""
from pathlib import Path
import numpy as np
import torch
from transformers import SegformerFeatureExtractor,SegformerConfig,SegformerForSemanticSegmentation
from semantic_floor import validate_rgb_packet,upright_rgb

def load_frozen(modelpath):
    torch.set_num_threads(1);torch.manual_seed(0);modelpath=Path(modelpath)
    model=SegformerForSemanticSegmentation(SegformerConfig.from_json_file(str(modelpath/'config.json')))
    model.load_state_dict(torch.load(modelpath/'pytorch_model.bin',map_location='cpu',weights_only=True),strict=True);model.eval();model.requires_grad_(False)
    processor=SegformerFeatureExtractor.from_pretrained(str(modelpath),local_files_only=True)
    return model,processor

def native_probabilities(packet,model,processor):
    validate_rgb_packet(packet);up=upright_rgb(packet['rgb']);inputs=processor(images=up,return_tensors='pt')
    with torch.inference_mode():
        logits=model(**inputs).logits;logits=torch.nn.functional.interpolate(logits,size=up.shape[:2],mode='bilinear',align_corners=False);prob=logits.softmax(dim=1)[0].numpy()
    return np.rot90(prob,-1,axes=(1,2)).copy()

def dense_head_probability(probabilities_native,rgb,head):
    prob=np.asarray(probabilities_native);rgb=np.asarray(rgb);w=np.asarray(head['weight'],float)
    if prob.ndim!=3 or prob.shape[0]!=150 or rgb.shape!=(*prob.shape[1:],3) or w.shape!=(153,) or not np.isfinite(prob).all() or not np.isfinite(w).all() or not np.isfinite(float(head['bias'])):raise ValueError('finite frozen semantic probabilities, matching RGB and153 head required')
    score=np.tensordot(w[:150],prob,axes=(0,0))+np.sum(w[150:]*rgb.astype(float)/255,axis=-1)+float(head['bias'])
    return 1/(1+np.exp(-np.clip(score,-80,80)))
