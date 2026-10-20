"""Frozen packets, descriptive pixel types only; renderer depth is evaluator-only."""
from pathlib import Path
import json,hashlib
import numpy as np
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';O=H/'structural_foreground_development_v1';out=H/'source_evidence_types_development_v1';out.mkdir(exist_ok=False)
seal=json.loads((O/'prediction_seal.json').read_text());inputs={}
for name,digest in seal['output_sha256'].items():assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest
info=json.loads((O/'predictions.json').read_text());rows=[]
for r in info['rows']:
 f=r['frame'];eye=r['eye'];paths=[O/r['file'],D/'source_inputs'/f'frame_{f:03d}.npz',D/'frames'/f'frame_{f:03d}.npz']
 for path in paths:inputs[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
 p=np.load(paths[0]);rgb=np.load(paths[1])[eye+'_rgb'];depth=np.load(paths[2])[eye+'_depth_m'];cad=p['CAD_mask'].astype(bool);selected=p['structural_hypothesis'].astype(bool)
 assert rgb.shape==(480,640,3) and depth.shape==cad.shape==selected.shape
 # Exact RGB black is appearance, never physical absence, obstacle or free evidence.
 black=np.max(rgb,axis=-1)==0;dark=np.max(rgb,axis=-1)<=16;valid=np.isfinite(depth)&(depth>0)
 rows.append(dict(frame=f,eye=eye,current_RGB_pixels=int(cad.size),CAD_excluded=int(cad.sum()),selected=int(selected.sum()),selected_CAD=int((selected&cad).sum()),selected_exact_black=int((selected&black).sum()),selected_dark_0_to_16=int((selected&dark).sum()),selected_nonblack=int((selected&~black).sum()),postseal_selected_GT_depth_valid=int((selected&valid).sum()),postseal_selected_GT_invalid_nonblack=int((selected&~valid&~black).sum()),postseal_all_nonCAD_GT_invalid_nonblack=int((~cad&~valid&~black).sum()),postseal_all_nonCAD_GT_valid=int((~cad&valid).sum())))
result=dict(rows=rows,black_definition='max RGB == 0; descriptive only',dark_definition='max RGB <=16; descriptive only, no gate or threshold sweep',CAD_geometry_exclusion_not_external_obstacle=True,renderer_validity_evaluator_only=True,GT_invalid_not_physical_absence=True,GT_valid_not_qualified_stereo_depth=True,no_reinference_or_actor=True,role='evidence_type_audit_not_source_admission',source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),input_sha256=inputs)
(out/'assessment.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(rows,indent=2))
