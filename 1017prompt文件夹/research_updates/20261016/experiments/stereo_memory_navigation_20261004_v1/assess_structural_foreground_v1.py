"""Evaluate sealed 2D MODEL hypotheses; GT cannot enter producer or actions."""
from pathlib import Path
import json,hashlib
import numpy as np
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';O=H/'structural_foreground_development_v1'
sealpath=O/'prediction_seal.json';seal=json.loads(sealpath.read_text())
for group in ['input_sha256','output_sha256','source_sha256']:
    for name,expected in seal[group].items():assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==expected,name
protocol=json.loads((H/'structural_foreground_protocol_v2.json').read_text());records=json.loads((O/'predictions.json').read_text())['rows'];assert len(records)==12
scene=H/'minimum_prefix_transition_sparse_membership_v2/scene_collision_shapes.json';shapes=json.loads(scene.read_text());target=next(x for x in shapes if x['path']=='/E1Detour/Divider/divider_06');y,x=np.indices((480,640));rays=np.stack(((x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones_like(x)),axis=-1);rows=[];GT_inputs={str(scene):hashlib.sha256(scene.read_bytes()).hexdigest()}
for row in records:
    f=row['frame'];eye=row['eye'];files=[D/'frames'/f'frame_{f:03d}.npz',D/'known_poses'/f'frame_{f:03d}.npz']
    for file in files:GT_inputs[str(file)]=hashlib.sha256(file.read_bytes()).hexdigest()
    depth=np.load(files[0])[eye+'_depth_m'];T=np.load(files[1])[eye+'_camera_usd_world'];R=T[:3,:3]@np.diag([1.,-1.,-1.])
    with np.errstate(invalid='ignore'):world=T[:3,3]+(rays*depth[...,None])@R.T
    valid=np.isfinite(world).all(-1)&np.isfinite(depth)&(depth>0);truth=np.zeros_like(valid);target_mask=np.zeros_like(valid)
    for shape in shapes:
        c=np.array(shape['position']);member=valid&(np.linalg.norm(world[...,:2]-c[:2],axis=-1)<=shape['radius']+.001)&(np.abs(world[...,2]-c[2])<=shape['height']/2+.001)
        truth|=member&(world[...,2]>.05)
        if shape['path']==target['path']:target_mask=member
    prediction=np.load(O/row['file']);positive=prediction['structural_hypothesis'];n=int(positive.sum());tp=int((positive&truth).sum());nt=int(target_mask.sum());hit=int((positive&target_mask).sum())
    rows.append(dict(frame=f,eye=eye,positive=n,true_positive_structural=tp,false_positive=n-tp,invalid_truth_positive=int((positive&~valid).sum()),precision=None if not n else tp/n,target_GT_raw_pixels=nt,target_hypothesis_hits=hit,target_recall=None if not nt else hit/nt,target_CAD_masked_pixels=int((target_mask&prediction['CAD_mask']).sum())))
count=sum(r['positive'] for r in rows);tp=sum(r['true_positive_structural'] for r in rows);precision=tp/count if count else 0.;recall=[]
for f in protocol['frames']:
    epoch=[r for r in rows if r['frame']==f];n=sum(r['target_GT_raw_pixels'] for r in epoch);hits=sum(r['target_hypothesis_hits'] for r in epoch);recall.append(dict(frame=f,GT_target_pixels=n,recall=hits/n if n else None))
gate=protocol['frozen_gates'];gates=dict(pixel_count=count>=gate['positive_pixels_min'],unconditional_precision=precision>=gate['precision_min'],each_nonempty_target_epoch_recall=all(r['recall']>=gate['target_recall_each_nonempty_frame_min'] for r in recall if r['GT_target_pixels']>0))
result=dict(decision='PASS_2D_DEVELOPMENT_HYPOTHESIS_ONLY_NO_ACTOR_ADMISSION' if all(gates.values()) else 'REJECT_ONE_FROZEN_STRUCTURAL_FOREGROUND_HYPOTHESIS',rows=rows,total_positive=count,unconditional_precision=precision,target_epoch_recalls=recall,gates=gates,GT_loaded_only_after_prediction_seal=True,prediction_seal_sha256=hashlib.sha256(sealpath.read_bytes()).hexdigest(),GT_input_sha256=GT_inputs,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),not_fresh=True,no_depth_occupancy_free_or_control_admission=True,old_source_and_worldmodel_rejections_preserved=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
