from pathlib import Path
import sys,json,hashlib
import numpy as np
sys.path.insert(0,'/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310')
W=Path(__file__).resolve().parents[2];H=W/'experiments/stereo_memory_navigation_20261004_v1';P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
for p in [P,H,W/'experiments/visible_surface_connectivity_20261004_v1']:sys.path.insert(0,str(p))
from stereo_geometry import restore_native,FOCAL,BASELINE
from timed_static_base_self_mask import TimedStaticBaseSelfMask
from masked_source_pair import paired_sources
from paired_texture_surface import PairedTextureSurfaceMemory
R=W/'experiments/visual_imu_navigation90_20261012_v1/e2_run';masker=TimedStaticBaseSelfMask();surface=PairedTextureSurfaceMemory();records=[];identities={};states=json.loads((R/'states.json').read_text())
for frame in [100,150,180,190,195]:
 files={name:R/name/f'frame_{frame:03d}.npz' for name in ['source_inputs','known_poses','source_predictions']};saved=R/'sensor_packets'/f'source_{frame:03d}.npz';pred=np.load(files['source_predictions']);poses=np.load(files['known_poses']);rgb=np.load(files['source_inputs']);disp=restore_native(pred['disparity_px']);time=float(pred['time_s']);prediction=dict(depth_m=pred['depth_m'],confidence=pred['confidence'],disparity_native_px=disp,frame=frame,time_s=time);pose={k:poses[k] for k in poses.files};pose['time_s']=float(pose['time_s'])
 proprio=np.zeros((5,17),dtype=np.float32);proprio[-1,9:12]=states[frame]['joints'];assert abs(states[frame]['time_s']-time)<1e-8
 _,packet,masks=paired_sources(prediction,pose,proprio,masker,time)
 old=np.load(saved)
 for key in old.files:assert np.array_equal(packet[key],old[key]),(frame,key)
 inp={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in list(files.values())+[saved]};identities.update(inp)
 record=surface.update(packet,rgb['left_rgb'],rgb['right_rgb'],time,inp,disp)
 y,x=np.indices(disp.shape);partner=y-np.where(np.isfinite(disp),disp,0);iy=np.rint(np.clip(partner,0,479)).astype(int)
 left_near=np.isfinite(disp)&(disp>FOCAL*BASELINE)&(disp>0)
 overlap=(partner>=0)&(partner<=479)
 row={'frame':frame,'time_s':time,'packet_reconstruction_exact':True,'saved_confident_depth_under1m':int((pred['confidence']&np.isfinite(pred['depth_m'])&(pred['depth_m']<1)).sum()),'left_raw_disparity_implying_under1m_pixels':int(left_near.sum()),'near_raw_outside_partner_image':int((left_near&~overlap).sum()),'near_raw_left_selfmasked':int((left_near&masks['left']).sum()),'near_raw_partner_selfmasked_in_overlap':int((left_near&overlap&masks['right'][iy,x]).sum()),'near_raw_unmasked_overlap_but_not_confident':int((left_near&overlap&~masks['left']&~masks['right'][iy,x]&~pred['confidence']).sum()),'source_packet_points':len(packet['world']),'height_prior_retained':int(((packet['world'][:,2]>.05)&(packet['world'][:,2]<=1.5)).sum()),'height_and_texture_retained':len(record['world'])};records.append(row)
print(json.dumps({'records':records,'input_sha256':identities,'scope':'Exact deterministic saved-output source-filter replay, no stereo matching/physical query. Raw near disparity not valid measurement; mask counters overlap and cannot be summed. Right raw disparity was not saved, LR failure vs right match invalidity cannot be disambiguated.','candidate_admitted':False}))
