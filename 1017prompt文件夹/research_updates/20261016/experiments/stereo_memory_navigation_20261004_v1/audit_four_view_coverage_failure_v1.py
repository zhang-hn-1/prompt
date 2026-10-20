"""Offline label diagnostic only, no new proposals/decisions or threshold selection."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
from scipy.spatial import cKDTree
H=Path(__file__).resolve().parent;D=H/'causal_four_view_new_region_v1';O=H/'qualified_hazard_navigation_run_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((D/'prediction_seal.json').read_text());assert all(sha(p)==h for p,h in seal['output_sha256'].items());assert (D/'assessment_v1.json').exists()
a=np.load(O/'sensor_packets/source_180.npz')['world'];a=a[(a[:,2]>.05)&(a[:,2]<=1.5)];seedbins=set(map(tuple,np.floor(a[:,:2]/.1).astype(int)));tree=cKDTree(a[:,:2]);rows=[]
for r in json.loads((D/'records.json').read_text()):
 i=r['frame'];src=np.load(O/'sensor_packets'/f'source_{i:03d}.npz');truth=np.load(O/'offline_truth'/f'frame_{i:03d}.npz');mask=np.load(O/'masks'/f'frame_{i:03d}.npz');pred=np.load(O/'source_predictions'/f'frame_{i:03d}.npz');w=src['world'];h=(w[:,2]>.05)&(w[:,2]<=1.5);p=w[h];eyes=[];sets=[]
 for eye in ['left','right']:
  yy,xx=np.mgrid[0:480:2,0:640:2];z=truth[eye+'_depth_m'][yy,xx];T=truth[eye+'_camera_usd_world'];R=T[:3,:3]@np.diag([1,-1,-1]);ray=np.stack([(xx-319.5)/584.8888888888889,(yy-239.5)/584.8888888888889,np.ones_like(xx)],axis=-1)
  with np.errstate(invalid='ignore'):world=T[:3,3]+(ray*z[:,:,None])@R.T
  valid=np.isfinite(z)&(z>=.5)&(z<=3)&~mask[eye][yy,xx]&(world[:,:,2]>.05)&(world[:,:,2]<=1.5)&(np.linalg.norm(world-truth['root_position_world_m'],axis=-1)>.55)
  ww=world[valid];b=np.floor(ww[:,:2]/.1).astype(int);novel=np.array([tuple(bi) not in seedbins for bi in b],bool);bs=set(map(tuple,b[novel]));sets.append(bs);item=dict(eye=eye,renderer_visible_hazard_pixels=len(ww),new_bin_hazard_pixels=int(novel.sum()),new_bins=len(bs),descriptive_distance_ge10cm=int((tree.query(ww[:,:2])[0]>=.1).sum()) if len(ww) else 0)
  if eye=='left':
   mz=pred['depth_m'][yy,xx][valid];conf=pred['confidence'][yy,xx][valid].astype(bool);item.update(model_confident_on_visible_hazard=int(conf.sum()),model_finite_in_range_on_visible_hazard=int((np.isfinite(mz)&(mz>=.5)&(mz<=3)).sum()),model_confident_and_range_on_visible_hazard=int((conf&np.isfinite(mz)&(mz>=.5)&(mz<=3)).sum()))
  eyes.append(item)
 errs=[]
 for eye,ids in [('left',src['query_pixel_id'][h]),('right',src['partner_pixel_id'][h])]:
  x,y=ids%640,ids//640;z=truth[eye+'_depth_m'][y,x];T=truth[eye+'_camera_usd_world'];ray=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))]
  with np.errstate(invalid='ignore'):world=T[:3,3]+(ray*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;errs.append(np.linalg.norm(world-p,axis=1))
 err=np.asarray(errs).T;correct=np.isfinite(err).all(1)&np.all(err<=.1,axis=1);bins=np.floor(p[:,:2]/.1).astype(int);new=np.array([tuple(b) not in seedbins for b in bins],bool)
 groups=[]
 for g in r['groups']:
  f=np.load(D/g['file']);valid=f['bilateral_fov_and_mask_valid'];std=valid&np.all(f['patch_std']>=5,axis=(1,2));temp=f['appearance_corroborated'];motion=min(f['both_camera_displacement_m'])>=.05;crossold=temp&(f['cross_eye_source_NCC']>=.8) if motion else np.zeros(len(temp),bool);crossnow=crossold&(f['cross_eye_current_NCC']>=.8);nb=np.array([tuple(b) not in seedbins for b in np.floor(f['past_world'][:,:2]/.1).astype(int)],bool)
  groups.append(dict(source_frame=g['source_frame'],proposals=len(valid),bilateral_projectable=int(valid.sum()),adequate_patch_std=int(std.sum()),temporal_NCC=int(temp.sum()),motion_prerequisite=bool(motion),plus_cross_eye_source=int(crossold.sum()),plus_cross_eye_current=int(crossnow.sum()),new_proposals=int(nb.sum()),new_projectable=int((nb&valid).sum()),new_temporal=int((nb&temp).sum()),new_confirmed=int((nb&f['confirmed_four_view']).sum())))
 rows.append(dict(frame=i,time_s=r['time_s'],renderer_eye_diagnostics=eyes,renderer_new_bins_seen_in_both_eyes=len(sets[0]&sets[1]),raw_hazards=len(p),raw_correct=int(correct.sum()),raw_new_hazards=int(new.sum()),raw_new_bilateral_correct=int((new&correct).sum()),groups=groups))
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,readonly_offline_causal_layer_diagnostic=True,renderer_visibility_not_controller_input=True,common_bins_not_exact_stereo_correspondence=True,no_new_rule_or_threshold=True,assessment_REJECT_unchanged=True,code_sha256=sha(__file__))
f=D/'coverage_failure_audit_v1.json';assert not f.exists();f.write_text(json.dumps(result,indent=2)+'\n')
for r in rows:
 if r['frame']>=180:print(r['frame'],'rawhaz/new/correctnew',r['raw_hazards'],r['raw_new_hazards'],r['raw_new_bilateral_correct'],'GTbothnewbins',r['renderer_new_bins_seen_in_both_eyes'],'GTleft',r['renderer_eye_diagnostics'][0])
