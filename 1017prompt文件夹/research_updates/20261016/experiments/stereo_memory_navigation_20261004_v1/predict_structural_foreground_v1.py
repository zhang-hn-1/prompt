"""ONE frozen structural segmentation MODEL hypothesis; seals before GT scoring."""
from pathlib import Path
import json,hashlib,time,importlib.metadata
import numpy as np
import torch
from PIL import Image
from safetensors.torch import load_file
from transformers import SegformerForSemanticSegmentation,SegformerConfig,SegformerFeatureExtractor
from timed_static_base_self_mask import TimedStaticBaseSelfMask
from dynamic_self_mask import joints_from_proprio
from structural_hypothesis_v1 import hypothesis_from_rotated
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';M=H/'third_party/segformer_b0_structural_v1';O=H/'structural_foreground_development_v1';O.mkdir(exist_ok=False)
p=H/'structural_foreground_protocol_v2.json';protocol=json.loads(p.read_text());identity=json.loads((M/'identity.json').read_text());assert identity['revision']==protocol['revision']
inputs={str(p):hashlib.sha256(p.read_bytes()).hexdigest(),str(M/'identity.json'):hashlib.sha256((M/'identity.json').read_bytes()).hexdigest()}
mask_identity=H.parent/'visible_surface_connectivity_20261004_v1/mask_identity.json'
mask_info=json.loads(mask_identity.read_text())
for file in [mask_identity,H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz']:
    inputs[str(file)]=hashlib.sha256(file.read_bytes()).hexdigest()
for file,expected in mask_info['source_layers_sha256'].items():
    actual=hashlib.sha256(Path(file).read_bytes()).hexdigest();assert actual==expected;inputs[file]=actual
for name,entry in identity['files'].items():
    file=M/name;assert hashlib.sha256(file.read_bytes()).hexdigest()==entry['sha256'];inputs[str(file)]=entry['sha256']
torch.set_num_threads(2);config=SegformerConfig.from_dict(json.loads((M/'config.json').read_text()));model=SegformerForSemanticSegmentation(config);model.load_state_dict(load_file(str(M/'model.safetensors')),strict=True);model.eval();processor=SegformerFeatureExtractor.from_dict(json.loads((M/'preprocessor_config.json').read_text()));cad=TimedStaticBaseSelfMask()
label_ids=[int(k) for k,v in config.id2label.items() if v in protocol['positive_labels']];assert label_ids
rows=[]
for frame in protocol['frames']:
    paths=[D/'source_inputs'/f'frame_{frame:03d}.npz',D/'history'/f'frame_{frame:03d}.npz']
    for file in paths:inputs[str(file)]=hashlib.sha256(file.read_bytes()).hexdigest()
    rgb=np.load(paths[0]);hist=np.load(paths[1])['proprio_history'].copy();hist[:,:9]=0;masks=cad.project(joints_from_proprio(hist))
    for eye in protocol['eyes']:
        native=rgb[eye+'_rgb'];rotated=np.rot90(native).copy();start=time.perf_counter()
        with torch.inference_mode():
            tensor=processor(images=Image.fromarray(rotated),return_tensors='pt');logits=model(**tensor).logits;logits=torch.nn.functional.interpolate(logits,size=rotated.shape[:2],mode='bilinear',align_corners=False);confidence,labels=torch.softmax(logits,dim=1).max(dim=1)
        result=hypothesis_from_rotated(labels[0].numpy(),confidence[0].numpy(),masks[eye],label_ids);elapsed=time.perf_counter()-start;name=f'frame_{frame:03d}_{eye}.npz'
        np.savez_compressed(O/name,label_native=result['label_native'],confidence_native=result['confidence_native'],structural_hypothesis=result['structural_hypothesis'],CAD_mask=masks[eye],capture_time_s=float(rgb['time_s']))
        rows.append(dict(frame=frame,eye=eye,file=name,inference_postprocess_wall_s=elapsed,positive_model_pixels=int(result['structural_hypothesis'].sum()),role=result['role'],observed_depth=False,certified_free=False))
assert len(rows)==protocol['maximum_image_inferences'];(O/'predictions.json').write_text(json.dumps(dict(rows=rows,positive_class_ids=label_ids,id2label=config.id2label,model_revision=identity['revision'],device='CPU',no_truth_labels_or_future_feedback=True,no_actor_install=True,not_learning_memory_increment=True),indent=2)+'\n')
files=[file for file in O.iterdir() if file.is_file()];sources=[Path(__file__),H/'structural_hypothesis_v1.py',H/'timed_static_base_self_mask.py',H/'dynamic_self_mask.py',H/'static_base_cached_self_mask_v2.py',H.parent/'fixed_rig_self_mask_and_curve_qualification_20261003_v1/self_mask.py']
(O/'prediction_seal.json').write_text(json.dumps(dict(input_sha256=inputs,output_sha256={str(file):hashlib.sha256(file.read_bytes()).hexdigest() for file in files},source_sha256={str(file):hashlib.sha256(file.read_bytes()).hexdigest() for file in sources},software_versions={name:importlib.metadata.version(name) for name in ['torch','transformers','safetensors','numpy','Pillow']},all_predictions_before_GTdepth_or_instance_labels=True),indent=2)+'\n');print([(r['frame'],r['eye'],r['positive_model_pixels'],round(r['inference_postprocess_wall_s'],3)) for r in rows])
