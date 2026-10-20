"""One source-only pre-LR ablation; no network inference or controller changes."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
from stereo_geometry import FOCAL,BASELINE,restore_native
from masked_source_pair import paired_sources
from dynamic_self_mask import RobotSelfMask
H=Path(__file__).resolve().parent;D=H/'neutral_bank_visibility_run_v1';A=H/'forward_disparity_qualification_ablation_v1';A.mkdir(exist_ok=False);cad=RobotSelfMask();states=json.loads((D/'states.json').read_text());ids=[200,210,215,220,225,230,235,240,245];sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();manifest={}
for i in ids:
 f=D/'source_predictions'/f'frame_{i:03d}.npz';s=np.load(f);raw=restore_native(s['disparity_px']);finite=np.isfinite(raw)&(raw>0);depth=np.full(raw.shape,np.nan,np.float32);depth[finite]=FOCAL*BASELINE/raw[finite];poses=np.load(D/'known_poses'/f'frame_{i:03d}.npz');history=np.load(D/'history'/f'frame_{i:03d}.npz');t=float(s['time_s']);assert float(poses['time_s'])==t
 _,packet,masks=paired_sources(dict(depth_m=depth,confidence=finite,disparity_native_px=raw,frame=i,time_s=t),dict(left_camera_usd_world=poses['left_camera_usd_world'],right_camera_usd_world=poses['right_camera_usd_world'],root_position_world_m=np.asarray(states[i]['position']),time_s=t),history['proprio_history'],cad,t)
 packet.update(left_right_consistency_checked=False,source_model_admitted=False,experimental_preLR_ablation=True)
 out=A/f'source_{i:03d}.npz';np.savez_compressed(out,**packet);manifest[str(out)]=sha(out)
 for p in [f,D/'known_poses'/f'frame_{i:03d}.npz',D/'history'/f'frame_{i:03d}.npz']:manifest[str(p)]=sha(p)
(A/'prediction_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),code_sha256=sha(__file__),artifact_and_input_sha256=manifest,GT_or_future_feedback_read=False,frames=ids),indent=2)+'\n');print('SEALED',len(ids),'no new inference')
