"""Post-close validation only: freshness, request/exposure completeness and frozen subset geometry."""
from pathlib import Path
import hashlib,json,datetime,numpy as np,cv2
from audit_stereo import summarize
from paired_texture_surface import PairedTextureSurfaceMemory
from masked_source_pair import paired_sources
from dynamic_self_mask import RobotSelfMask
from stereo_geometry import FOCAL,restore_native
H=Path(__file__).resolve().parent;O=H/'simple_E1_partial_source_probe_run_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
status=json.loads((H/'simple_E1_partial_source_probe_stage_status_v1.json').read_text());assert status['exit_code']==0 and not status['wall_timeout']
seal=O/'post_close_manifest.json'
# seal_existing uses its own returned manifest name; locate recorded digest explicitly.
candidates=[p for p in O.glob('*.json') if sha(p)==status['post_close_manifest_sha256']];assert len(candidates)==1
seal=candidates[0]
sealed=json.loads(seal.read_text());assert sealed['complete'] and not sealed['wall_timeout']
for p,d in sealed['artifact_sha256'].items():assert sha(p)==d,p
identity=json.loads((O/'identity.json').read_text());assert identity['complete']
for group in ('source_sha256','input_sha256'):
 for p,d in identity[group].items():assert sha(p)==d,p
np.testing.assert_array_equal(np.load(O/'requests.npz')['requests'],np.load(H/'simple_E1_frozen_openloop_requests_v1.npz')['requests'])
clocks=json.loads((O/'camera_clocks.json').read_text());states=json.loads((O/'states.json').read_text());rows=json.loads((O/'source_rows.json').read_text())
assert len(clocks)==len(states)==141 and len(rows)==29
phys=np.load(O/'physics.npz');assert len(phys['simulation_time_s'])==701
np.testing.assert_allclose([r['episode_time_s'] for r in clocks],np.arange(141)*.1,atol=1e-5,rtol=0)
# Independent original CAD/source reconstruction before reading depth labels.
cad=RobotSelfMask();memory=PairedTextureSurfaceMemory()
for row in rows:
 i=row['frame'];name=f'frame_{i:03d}.npz';pose=np.load(O/'known_poses'/name);pred=np.load(O/'source_predictions'/name);rgb=np.load(O/'source_inputs'/name);packet=np.load(O/'sensor_packets'/f'source_{i:03d}.npz');history=np.load(O/'history'/name)['proprio_history'].copy();history[:,:9]=0.;t=float(pose['time_s'])
 _,rebuilt,_=paired_sources(dict(depth_m=pred['depth_m'],confidence=pred['confidence'],disparity_native_px=restore_native(pred['disparity_px']),frame=i,time_s=t),dict(**{key:pose[key] for key in ['left_camera_usd_world','right_camera_usd_world','root_position_world_m','time_s']}),history,cad,t)
 for key in packet.files:np.testing.assert_array_equal(rebuilt[key],packet[key])
 surface=memory.update(packet,rgb['left_rgb'],rgb['right_rgb'],t,row['provenance'],restore_native(pred['disparity_px']));saved=np.load(O/'partial_observations'/name)
 for key in ['world','sigma_m','query_pixel_id','partner_pixel_id']:np.testing.assert_array_equal(surface[key],saved[key])
 assert not saved['certified_free'] and not saved['unseen_volume_observed']
reconstruction=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),exposures=29,point_fields_exact=116,original_CAD_rebuild=True,GT_pose_upperbound=True,code_sha256=sha(__file__),post_close_seal_sha256=sha(seal))
(H/'simple_E1_source_reconstruction_pretruth_v1.json').write_text(json.dumps(reconstruction,indent=2)+'\n')
errors=[];bands=[];scores=[]
for r in rows:
 i=r['frame'];assert i%5==0
 for p,d in r['provenance'].items():assert sha(p)==d,p
 p=np.load(O/'partial_observations'/f'frame_{i:03d}.npz');gt=np.load(O/'frames'/f'frame_{i:03d}.npz');pose=np.load(O/'known_poses'/f'frame_{i:03d}.npz')
 for key in ['left_camera_usd_world','right_camera_usd_world','root_position_world_m','time_s']:np.testing.assert_array_equal(pose[key],gt[key])
 assert abs(float(gt['time_s'])-clocks[i]['episode_time_s'])<1e-8
 y,x=np.divmod(p['query_pixel_id'],640);z=gt['left_depth_m'][y,x];rays=np.column_stack(((x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(len(x))));T=pose['left_camera_usd_world'];w=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
 valid=np.isfinite(z)&(z>0)&np.isfinite(w).all(axis=1);e=np.linalg.norm(p['world']-w,axis=1);e[~valid]=np.inf
 errors.append(e);bands.append(valid&(w[:,2]>.05)&(w[:,2]<=1.5));scores.append(dict(frame=i,n=len(e),metric=summarize(e)))
e=np.concatenate(errors);metric=summarize(e);checks=dict(n=len(e)>=100,fraction95=metric['within_0p10m'] is not None and metric['within_0p10m']>=.95,p90=metric['unconditional_p90_m'] is not None and metric['unconditional_p90_m']<=.1,two100=sum(len(e)>=100 for e in errors)>=2)
videos={}
for name in ['stereo_view','third_person']:
 cap=cv2.VideoCapture(str(O/(name+'.mp4')));n=0
 while cap.read()[0]:n+=1
 cap.release();assert n==141;videos[name]=n
contacts=json.loads((O/'physics_contacts.json').read_text());positions=np.asarray([r['position'] for r in states]);a=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_FRESH_ORIGINAL_E1_SELECTED_GEOMETRY_KNOWN_POSE_ONLY' if all(checks.values()) else 'REJECT_FRESH_ORIGINAL_E1_SELECTED_GEOMETRY',checks=checks,metric=metric,GT_height_band_precision=float(np.concatenate(bands).mean()) if len(e) else None,not_full_semantic_or_coverage_gate=True,GT_pose_upperbound=True,requests=140,captures=141,physics=701,exposures=29,videos=videos,post_close_seal_sha256=sha(seal),path_length_m=float(np.linalg.norm(np.diff(positions[:,:2],axis=0),axis=1).sum()),displacement_m=float(np.linalg.norm(positions[-1,:2]-positions[0,:2])),contacts_samples=len(contacts),not_navigation=True,not_original35=True,whole_research_complete=False)
D=H/'simple_E1_partial_source_probe_assessment_v1';D.mkdir(exist_ok=False);(D/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');(D/'exposure_scores.json').write_text(json.dumps(scores,indent=2)+'\n');print(json.dumps(a,indent=2))
