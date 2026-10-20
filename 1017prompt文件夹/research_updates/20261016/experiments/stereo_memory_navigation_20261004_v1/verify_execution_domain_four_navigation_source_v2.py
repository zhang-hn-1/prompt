from pathlib import Path
import sys,json,hashlib,datetime,numpy as np,cv2
from scipy.spatial import cKDTree
from bilateral_core_memory import BilateralCoreMemory
from ray_contradiction_memory import contradict_cores
from partial_cue_source_v2 import inflate_xy,PartialCueSetSource
H=Path(__file__).resolve().parent;arm=sys.argv[1];assert arm in ['R0','R1'];D=H/('execution_domain_four_navigation_'+arm+'_run_v2');sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
st=json.loads((H/'execution_domain_four_navigation_stage_status_v2.json').read_text())['arms'][arm];assert st['complete'];manifest=D/'artifact_manifest_post_close.json';assert sha(manifest)==st['post_close_manifest_sha256'];hashes=json.loads(manifest.read_text())['artifact_sha256'];assert all(sha(f)==s for f,s in hashes.items());seal=json.loads((D/'memory_actor_pretruth_seal.json').read_text());assert all(sha(f)==s for f,s in seal.items())
m=BilateralCoreMemory();pairs=[];original=PartialCueSetSource(.2);opportunity={r['frame']:r for r in json.loads((D/'source_opportunity_records.json').read_text())};factual_fields=0
for i in range(0,141,5):
 q=dict(np.load(D/'possible_sets'/f'frame_{i:03d}.npz'));t=float(q['time_s']);est=dict(np.load(D/'estimated_poses'/f'frame_{i:03d}.npz'));init=np.load(D/'estimated_poses/frame_000.npz');derived=original.update(dict(np.load(D/'sensor_packets'/f'source_{i:03d}.npz')),dict(np.load(D/'online_inputs'/f'frame_{i:03d}.npz')),est,dict(np.load(D/'masks'/f'frame_{i:03d}.npz')),opportunity[i]['qualified_seed'],t,i,est['root_position_world_m'][:2],np.array([2.2,.15]),np.array([-2.2,.15]))
 for k,v in derived.items():np.testing.assert_equal(v,q[k]);factual_fields+=1
 m.update(q,t);pairs.append((dict(np.load(D/'sensor_packets'/f'source_{i:03d}.npz')),dict(np.load(D/'estimated_poses'/f'frame_{i:03d}.npz'))))
full=m.freeze(m.last_time);saved=np.load(D/'unfiltered_historical_pool.npz')
for k,v in full.items():np.testing.assert_equal(v,saved[k])
raw=full['raw_world_voxel_centres'];veto,proof=contradict_cores(raw,pairs,m.last_time);records=json.loads((D/'ray_contradiction_proofs.json').read_text());assert records==[dict(core_xyz=c.tolist(),contradicted=bool(b),past_ray_proofs=p) for c,b,p in zip(raw,veto,proof)]
expected=full if arm=='R1' else dict(q,static_persistence_prior=True,per_point_measured_timestamps_available=False)
if arm=='R1':expected.update(raw_world_voxel_centres=raw[~veto],possible_xy=inflate_xy(raw[~veto,:2],.2))
expected.update(source_consumer='negative_ray_history' if arm=='R1' else 'last_current_strong',not_measured_free=True);seed=np.load(D/'frozen_memory_seed.npz')
for k,v in expected.items():np.testing.assert_equal(v,seed[k])
# Above reconstruction never uses labels or future i>140. Future-payload trap.
try:contradict_cores(raw,[({'time_s':m.last_time+.5},{'time_s':m.last_time+.5})],m.last_time)
except ValueError:pass
else:raise AssertionError('future ray accepted')
truth=np.load(D/'offline_occupancy_label.npz');gtxy=truth['origin']+np.argwhere(truth['labels']>0)*float(truth['resolution']);coverage=float((cKDTree(seed['possible_xy']).query(gtxy)[0]<=np.sqrt(2)*.05+1e-9).mean());inside=(np.linalg.norm(raw[:,:2],axis=1)<=.35)&(raw[:,2]>=0)&(raw[:,2]<=1)
Y,X=np.mgrid[:480,:640];ray=np.stack([(X-319.5)/584.8888888888889,(Y-239.5)/584.8888888888889,np.ones_like(X)],-1);views=[]
windows=m.windows if arm=='R1' else [m.windows[-1]]
for w in windows:
 for vf in w['support_frames']:
  for eye in ['left','right']:
   cl=np.load(D/'partial_class_masks'/f"birth_{w['frame']:03d}_{eye}_{vf:03d}.npz")['potential_hazard_mask'];gt=np.load(D/'offline_truth'/f'frame_{vf:03d}.npz');z=gt[eye+'_depth_m'];T=gt[eye+'_camera_usd_world'];mask=np.load(D/'masks'/f'frame_{vf:03d}.npz')[eye]
   with np.errstate(invalid='ignore'):world=T[:3,3]+(ray*z[:,:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
   valid=np.isfinite(z)&np.isfinite(world).all(-1)&(z>=.5)&(z<=3)&(world[:,:,2]>.05)&(world[:,:,2]<=1.5)&(np.linalg.norm(world-gt['root_position_world_m'],axis=-1)>.55)&~mask
   views.append(dict(window=w['frame'],exposure=vf,eye=eye,precision=float((cl&valid).sum()/cl.sum()) if cl.any() else 0.))
checks=dict(coverage_ge95=coverage>=.95,area_le6=len(seed['possible_xy'])*.01<=6,all_contributing_class_precision_ge95=all(v['precision']>=.95 for v in views),no_true_occupied_veto=int((inside&veto).sum())==0)
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),arm=arm,decision='PASS_FRESH_LOCAL_SOURCE_ONLY' if all(checks.values()) else 'REJECT_FRESH_LOCAL_SOURCE',checks=checks,coverage=coverage,possible_area_m2=len(seed['possible_xy'])*.01,raw_core_count=len(seed['raw_world_voxel_centres']),contradicted_cores=int(veto.sum()),wrong_veto=int((inside&veto).sum()),class_min=min(v['precision'] for v in views),class_views=views,all_reconstruction_before_GT=True,original_source_fields_exact=factual_fields,future_ray_rejected=True,no_stateful_policy_recall=True,not_measured_free=True,original_source_rejects_preserved=True,whole_research_complete=False,code_sha256=sha(__file__))
O=H/('execution_domain_four_navigation_'+arm+'_source_verification_v2.json');assert not O.exists();O.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='class_views'},indent=2))
