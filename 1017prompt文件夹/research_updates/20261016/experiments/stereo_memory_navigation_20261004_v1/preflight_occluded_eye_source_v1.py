from pathlib import Path
import json,hashlib,datetime,numpy as np
from scipy.spatial import cKDTree
from occluded_eye_partial_source import OccludedEyePartialSource
H=Path(__file__).resolve().parent;O=H/'occluded_eye_preflight_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();rows=[];preds={}
for case,y,birthcheck in [('flat_ground_pose_D3_run_v1',0.,'flat_ground_pose_birth_source_check_v1.json'),('offset_replication_D4_run_v1',.15,'offset_replication_birth_source_check_v1.json')]:
 D=H/case;man=json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256'];assert all(sha(f)==s for f,s in man.items());seed=np.load(D/'immutable_possible_set_birth.npz');end=int(seed['frame']);source=OccludedEyePartialSource(.2);fields=0
 for r in json.loads((D/'source_opportunity_records.json').read_text()):
  i=r['frame']
  if i>end:break
  est=dict(np.load(D/'estimated_poses'/f'frame_{i:03d}.npz'));a=source.update(dict(np.load(D/'sensor_packets'/f'source_{i:03d}.npz')),dict(np.load(D/'online_inputs'/f'frame_{i:03d}.npz')),est,dict(np.load(D/'masks'/f'frame_{i:03d}.npz')),r['qualified_seed'],r['time_s'],i,est['root_position_world_m'][:2],np.array([2.2,y]),np.array([-2.2,y]));old=np.load(D/'possible_sets'/f'frame_{i:03d}.npz')
  for k in source.original_packet:np.testing.assert_equal(source.original_packet[k],old[k]);fields+=1
 oldxy=source.original_packet['possible_xy'];assert (cKDTree(a['possible_xy']).query(oldxy)[0]<1e-9).all()
 np.savez_compressed(O/(case+'.npz'),**a);preds[case]=a;rows.append(dict(case=case,factual_fields_exact=fields,strong_source_subset_preserved=True,original_area_m2=len(oldxy)*.01,source_birth_frame=end,birthcheck_file=birthcheck))
(O/'outputs_pretruth_seal.json').write_text(json.dumps({str(f):sha(f) for f in O.glob('*.npz')},indent=2)+'\n')
for r in rows:
 D=H/r['case'];a=preds[r['case']];truth=np.load(D/'offline_occupancy_label.npz');xy=truth['origin']+np.argwhere(truth['labels']>0)*float(truth['resolution']);coverage=float((cKDTree(a['possible_xy']).query(xy)[0]<=np.sqrt(2)*.05+1e-9).mean());oldcheck=json.loads((H/r['birthcheck_file']).read_text());classpass=oldcheck['checks']['birth_view_precision_ge95'];r.update(coverage=coverage,possible_area_m2=len(a['possible_xy'])*.01,checks=dict(coverage_ge95=coverage>=.95,area_le6=len(a['possible_xy'])*.01<=6,class_precision_unchanged_ge95=classpass))
passed=all(all(r['checks'].values()) for r in rows)
out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_HISTORICAL_PREFLIGHT_ONLY' if passed else 'REJECT_UNIQUE_OCCLUDED_EYE_COMBINER',rows=rows,no_GT_actor_or_future=True,historical_not_fresh=True,original_D4_and_old_source_reject_preserved=True,no_radius_threshold_or_network_scan=True,actual_navigation_triggered=False,whole_research_complete=False,source_sha256={str(H/n):sha(H/n) for n in ['occluded_eye_support.py','occluded_eye_partial_source.py','preflight_occluded_eye_source_v1.py']});(O/'assessment.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
