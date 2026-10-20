"""ALL postclose exposures; GT-only ownership of observed stereo overlap loss."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
H=Path(__file__).resolve().parent;D=H/'camera_heading_observation_run_v1'
A=H/'camera_heading_overlap_loss_v1';A.mkdir(exist_ok=False)
assessment=json.loads((H/'camera_heading_observation_assessment_v1.json').read_text())
assert assessment['decision']=='REJECT_CAMERA_HEADING_OBSERVATION_PROBE'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
base=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
inputs={};rows=[];F=584.8888888888889;B=.6;yy,xx=np.mgrid[0:480:2,0:640:2];Zmin=F*B/479
for item in assessment['all_window_source_updates']:
 i=item['frame'];tp=D/'offline_truth'/f'frame_{i:03d}.npz';mp=D/'masks'/f'frame_{i:03d}.npz';sp=D/'sensor_packets'/f'source_{i:03d}.npz'
 for p in [tp,mp,sp]:inputs[str(p)]=sha(p)
 truth=np.load(tp);mask=np.load(mp);z=truth['left_depth_m'][yy,xx];T=truth['left_camera_usd_world'];R=T[:3,:3]@np.diag([1,-1,-1]);ray=np.stack([(xx-319.5)/F,(yy-239.5)/F,np.ones_like(xx)],axis=-1)
 with np.errstate(invalid='ignore'):w=T[:3,3]+(ray*z[:,:,None])@R.T
 valid=np.isfinite(z)&(z>=.5)&(z<=3)&~mask['left'][yy,xx]&(w[:,:,2]>.05)&(w[:,:,2]<=1.5)&(np.linalg.norm(w-truth['root_position_world_m'],axis=-1)>.55)
 world=w[valid];lx=xx[valid];ly=yy[valid];lz=z[valid];T=truth['right_camera_usd_world'];R=T[:3,:3]@np.diag([1,-1,-1]);local=(world-T[:3,3])@R
 with np.errstate(divide='ignore',invalid='ignore'):uv=local[:,:2]/local[:,2,None]*F+[319.5,239.5]
 positive=(local[:,2]>0)&np.isfinite(uv).all(1);inX=positive&(uv[:,0]>=0)&(uv[:,0]<639.5);inY=positive&(uv[:,1]>=0)&(uv[:,1]<479.5);fov=inX&inY
 ix=np.flatnonzero(fov);u,v=np.rint(uv[ix]).astype(int).T;fixed=base['right'][v,u];blocked=mask['right'][v,u];assert not np.any(fixed&~blocked)
 rz=truth['right_depth_m'][v,u];rayr=np.c_[(u-319.5)/F,(v-239.5)/F,np.ones(len(u))]
 with np.errstate(invalid='ignore'):rw=T[:3,3]+(rayr*rz[:,None])@R.T;err=np.linalg.norm(rw-world[ix],axis=1)
 common=~blocked&np.isfinite(err)&(err<=.1)
 difference=np.abs(uv[:,1]-(ly-F*B/lz));difX=np.abs(uv[:,0]-lx)
 pred=np.load(sp);haz=(pred['world'][:,2]>.05)&(pred['world'][:,2]<=1.5);q=pred['query_pixel_id'][haz];px=q%640;py=q//640;qz=truth['left_depth_m'][py,px];errs=[]
 for eye,k in [('left','query_pixel_id'),('right','partner_pixel_id')]:
  ids=pred[k][haz];x=ids%640;y=ids//640;zz=truth[eye+'_depth_m'][y,x];Tf=truth[eye+'_camera_usd_world'];rr=np.c_[(x-319.5)/F,(y-239.5)/F,np.ones(len(x))]
  with np.errstate(invalid='ignore'):actual=Tf[:3,3]+(rr*zz[:,None])@(Tf[:3,:3]@np.diag([1,-1,-1])).T;errs.append(np.linalg.norm(actual-pred['world'][haz],axis=1))
 errors=np.asarray(errs).T;correct=np.isfinite(errors).all(1)&(errors<=.1).all(1);wrong=np.isfinite(errors).all(1)&~correct
 assert int(wrong.sum())==item['finite_wrong'] and int(correct.sum())==item['correct']
 rows.append(dict(frame=i,time_s=float(truth['time_s']),left_current_visible=len(world),right_nonpositive_or_invalid=int((~positive).sum()),right_X_out=int((positive&~inX).sum()),right_Y_out=int((positive&~inY).sum()),right_either_XY_out=int((positive&~fov).sum()),right_in_fov=len(ix),fixed_base_CAD_blocked=int(fixed.sum()),additional_dynamic_CAD_blocked=int((blocked&~fixed).sum()),right_unmasked=int((~blocked).sum()),right_unmasked_same_surface=int(common.sum()),right_unmasked_surface_insufficient=int((~blocked&~np.isfinite(err)).sum()),right_unmasked_finite_surface_mismatch=int((~blocked&np.isfinite(err)&(err>.1)).sum()),left_below_optimistic_stereo_Zmin=int((lz<Zmin).sum()),left_axial_depth_median_m=float(np.median(lz)) if len(lz) else None,native_vertical_disparity_formula_max_error_px=float(difference.max()) if len(difference) else None,native_horizontal_coordinate_max_error_px=float(difX.max()) if len(difX) else None,emitted_hazards=len(q),finite_wrong=int(wrong.sum()),wrong_true_left_depth_below_Zmin=int((wrong&np.isfinite(qz)&(qz<Zmin)).sum()),wrong_true_right_partner_y_out=int((wrong&np.isfinite(qz)&((py-F*B/qz<0)|(py-F*B/qz>479))).sum())))
out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),GT_only_posthoc=True,not_online_AB=True,no_source_rescoring=True,whole_research_complete=False,formal_navigation_triggered=False,optimistic_native_Zmin_m=Zmin,center_row_Zmin_m=F*B/239.5,not_reliable_depth_range_or_collision_certificate=True,all_window_rows=rows,input_sha256=inputs,code_sha256=sha(__file__),plan_sha256=sha(H.parent.parent/'docs/superpowers/plans/2026-10-07-camera-heading-overlap-loss-diagnosis.md'))
p=A/'overlap_loss_ownership_v1.json';p.write_text(json.dumps(out,indent=2)+'\n');print([(r['frame'],r['left_current_visible'],r['right_either_XY_out'],r['fixed_base_CAD_blocked'],r['additional_dynamic_CAD_blocked'],r['right_unmasked_same_surface']) for r in rows])
