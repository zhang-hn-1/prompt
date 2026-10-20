"""Bounded postseal class/floor/gravity diagnosis, no classifier changes."""
from pathlib import Path
import numpy as np,json,hashlib
from PIL import Image,ImageDraw
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';O=H/'structural_foreground_development_v1';seal=json.loads((O/'prediction_seal.json').read_text())
for name,s in seal['output_sha256'].items():assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==s
info=json.loads((O/'predictions.json').read_text());shape=next(s for s in json.loads((H/'minimum_prefix_transition_sparse_membership_v2/scene_collision_shapes.json').read_text()) if s['path']=='/E1Detour/Divider/divider_06');c=np.array(shape['position']);y,x=np.indices((480,640));rays=np.stack(((x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones_like(x)),axis=-1);rows=[]
for r in info['rows']:
    f=r['frame'];eye=r['eye'];p=np.load(O/r['file']);depth=np.load(D/'frames'/f'frame_{f:03d}.npz')[eye+'_depth_m'];T=np.load(D/'known_poses'/f'frame_{f:03d}.npz')[eye+'_camera_usd_world'];R=T[:3,:3]@np.diag([1.,-1.,-1.])
    with np.errstate(invalid='ignore'):world=T[:3,3]+(rays*depth[...,None])@R.T
    valid=np.isfinite(world).all(-1)&np.isfinite(depth)&(depth>0);floor=valid&(np.abs(world[...,2])<=.05);target=valid&(np.linalg.norm(world[...,:2]-c[:2],axis=-1)<=shape['radius']+.001)&(np.abs(world[...,2]-c[2])<=shape['height']/2+.001);positive=p['structural_hypothesis'];gravity=R.T@np.array([0.,0.,1.]);rotated_xy=np.array([gravity[1],-gravity[0]])
    classes,counts=np.unique(p['label_native'][target],return_counts=True);target_classes=[dict(label=info['id2label'][str(int(i))],pixels=int(n)) for i,n in zip(classes,counts)]
    rows.append(dict(frame=f,eye=eye,positive=int(positive.sum()),positive_GT_floor=int((positive&floor).sum()),positive_GT_invalid=int((positive&~valid).sum()),GT_target_class_histogram=target_classes,gravity_up_xy_after_fixed_rotation=rotated_xy.tolist(),rotation_makes_gravity_up=bool(rotated_xy[1]<0 and abs(rotated_xy[1])>abs(rotated_xy[0]))))
    if eye=='left' and f in [215,330]:
        rgb=np.load(D/'source_inputs'/f'frame_{f:03d}.npz')['left_rgb'];overlay=rgb.copy();overlay[positive]=((overlay[positive].astype(float)+[255,0,0])/2).astype(np.uint8);overlay[target]=((overlay[target].astype(float)+[0,255,0])/2).astype(np.uint8)
        canvas=Image.new('RGB',(1280,510),'white');canvas.paste(Image.fromarray(rgb),(0,30));canvas.paste(Image.fromarray(overlay),(640,30));ImageDraw.Draw(canvas).text((8,8),'Raw current LEFT | MODEL structural red, postseal GT target green (not model input)',fill='black');canvas.save(O/f'frame_{f:03d}_failure_overlay.png')
result=dict(rows=rows,no_model_or_threshold_change=True,no_new_inference=True,GT_postseal_only=True,whole_research_complete=False,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest());(O/'failure_cause_audit_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(rows,indent=2))
