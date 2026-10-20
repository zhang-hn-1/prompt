"""Fixed classical matcher development probe; no labels or controller calls."""
from pathlib import Path
import hashlib,json,time
import numpy as np
from stereo_geometry import SGBMStereo,restore_native
from timed_static_base_self_mask import TimedStaticBaseSelfMask
from dense_source_points import point_packet
from paired_texture_surface import PairedTextureSurfaceMemory

H=Path(__file__).resolve().parent
cfg=json.loads((H/'sgbm_baseline_e2_six_exposure_protocol_v1.json').read_text())
D=Path(cfg['source_run']);O=H/cfg['outputs'];O.mkdir(exist_ok=False)
seal=json.loads((D/'artifact_manifest_post_close.json').read_text())
assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in seal['artifact_sha256'].items())
ph=np.load(D/'physics.npz');cad=TimedStaticBaseSelfMask();matcher=SGBMStereo();memory=PairedTextureSurfaceMemory();rows=[];inputs={}
for i in cfg['frames']:
    f=f'frame_{i:03d}.npz';rgbp=D/'source_inputs'/f;posep=D/'known_poses'/f
    inputs.update({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (rgbp,posep,D/'physics.npz',D/'partial_observations'/f)})
    with np.load(rgbp) as rgb,np.load(posep) as poses,np.load(D/'partial_observations'/f) as old:
        t=float(rgb['time_s']);assert t==float(poses['time_s'])
        assert abs(float(ph['simulation_time_s'][i*5])-t)<1e-8
        masks=cad.project(ph['joint_position_rad'][i*5])
        begin=time.monotonic();pred=matcher.predict(rgb['left_rgb'],rgb['right_rgb'],cad.frozen['left'],cad.frozen['right']);wall=time.monotonic()-begin
        predp=O/f'sgbm_prediction_{i:03d}.npz';np.savez_compressed(predp,**pred,frame=i,time_s=t)
        packet=point_packet(dict(depth_m=pred['depth_m'],confidence=pred['confidence'],disparity_native_px=restore_native(pred['disparity_px']),frame=i,time_s=t),dict(poses),masks['left'],masks['right'])
        np.savez_compressed(O/f'packet_{i:03d}.npz',**packet)
        surface=memory.update(packet,rgb['left_rgb'],rgb['right_rgb'],t,{str(predp):hashlib.sha256(predp.read_bytes()).hexdigest(),**{str(p):inputs[str(p)] for p in (rgbp,posep)}},restore_native(pred['disparity_px']))
        np.savez_compressed(O/f'partial_surface_{i:03d}.npz',**{k:v for k,v in surface.items() if k!='input_hashes'})
        cells=lambda points:set(map(tuple,np.rint(points[:,:2]/.1).astype(int)))
        newcells=cells(surface['world']);oldcells=cells(old['world'])
        row=dict(frame=i,time_s=t,sgbm_wall_s=wall,old_source_points=len(old['world']),new_source_points=len(surface['world']),old_cells=len(oldcells),new_cells=len(newcells),cell_symmetric_difference=len(newcells^oldcells),partial_risk_not_free_or_volume=True)
        rows.append(row);print(row,flush=True)
r=dict(protocol=cfg,rows=rows,old_nonempty_epochs=sum(a['old_source_points']>0 for a in rows),new_nonempty_epochs=sum(a['new_source_points']>0 for a in rows),different_nonempty_risk_epochs=sum(a['new_cells']>0 and a['cell_symmetric_difference']>0 for a in rows),new_inference_method='fixed existing SGBM only, no neural inference or fitting',not_fresh=True,not_online_interface_validation=True,not_navigation_acceptance=True,GT_depth_read=False,inputs_sha256=inputs)
r['functional_source_trial_justified']=r['different_nonempty_risk_epochs']>0
(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n')
(O/'artifact_manifest_post_close.json').write_text(json.dumps(dict(artifact_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in O.rglob('*') if p.is_file()}),indent=2)+'\n')
