"""Fit one historical model, seal fresh predictions before offline teacher reads."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from scipy.spatial import cKDTree
from hypothesis_logistic_v1 import fit_historical,predict,FIT
from hypothesis_features_v1 import scored_hypotheses
H=Path(__file__).resolve().parent;F=H/'hypothesis_learning_features_v1';O=H/'hypothesis_learning_model_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
def labels(case,w):
 truth=np.load(H/case/'offline_occupancy_label.npz');cells=np.rint((w[:,:2]-truth['origin'])/float(truth['resolution'])).astype(int);inside=((cells>=0)&(cells<np.asarray(truth['labels'].shape))).all(1);y=np.zeros(len(w),int);y[inside]=(truth['labels'][tuple(cells[inside].T)]>0);return y,truth

def class_precision(case,frame):
 D=H/case;Y,X=np.mgrid[:480,:640];ray=np.stack([(X-319.5)/584.8888888888889,(Y-239.5)/584.8888888888889,np.ones_like(X)],-1);rows=[]
 for f in sorted((D/'partial_class_masks').glob(f'birth_{frame:03d}_*.npz')):
  eye,vf=f.stem.split('_')[-2:];vf=int(vf);cl=np.load(f)['potential_hazard_mask'];gt=np.load(D/'offline_truth'/f'frame_{vf:03d}.npz');z=gt[eye+'_depth_m'];T=gt[eye+'_camera_usd_world'];mask=np.load(D/'masks'/f'frame_{vf:03d}.npz')[eye]
  with np.errstate(invalid='ignore'):w=T[:3,3]+(ray*z[:,:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
  valid=np.isfinite(z)&np.isfinite(w).all(-1)&(z>=.5)&(z<=3)&(w[:,:,2]>.05)&(w[:,:,2]<=1.5)&(np.linalg.norm(w-gt['root_position_world_m'],axis=-1)>.55)&~mask;n=int(cl.sum());rows.append(dict(frame=vf,eye=eye,pixels=n,precision=float((cl&valid).sum()/n) if n else 0.))
 return rows

def main():
 acq=json.loads((H/'hypothesis_learning_acquisition_status_v1.json').read_text());assert acq['stage']=='TWO_WHOLE_SEQUENCES_SEALED_BEFORE_FIT';seal=json.loads((F/'features_pretruth_seal.json').read_text());assert all(sha(p)==s for p,s in seal['output_sha256'].items());assert all(sha(p)==s for p,s in seal['code_sha256'].items());O.mkdir(exist_ok=False)
 metadata=dict(utc=now(),fit_started_after_held_sealed=True,acquisition_status_sha256=sha(H/'hypothesis_learning_acquisition_status_v1.json'),features_seal_sha256=sha(F/'features_pretruth_seal.json'),protocol_sha256=sha(H/'hypothesis_learning_stage_protocol_v1.json'),measurement_semantics_sha256=sha(H/'hypothesis_learning_measurement_semantics_v1.json'),code_sha256={str(H/n):sha(H/n) for n in ['run_hypothesis_learning_model_v1.py','hypothesis_logistic_v1.py','hypothesis_features_v1.py']},only_one_structure=True);(O/'fit_started.json').write_text(json.dumps(metadata,indent=2)+'\n')
 assert {r['case'] for r in seal['rows']}==FIT|{'hypothesis_learning_H1_run_v1','hypothesis_learning_H2_run_v1'} and len(seal['rows'])==4, 'exact whole-sequence split'
 data={r['case']:dict(np.load(F/(r['case']+'.npz'))) for r in seal['rows']};train={case:(data[case]['features'],labels(case,data[case]['voxels'])[0]) for case in sorted(FIT)};model=fit_historical(train);np.savez_compressed(O/'model.npz',**model);outputs={}
 for case,d in data.items():
  probabilities=predict(model,d['features']);a=scored_hypotheses(d['voxels'],probabilities,float(d['time_s']));a.update(probabilities=probabilities,all_voxels=d['voxels'],frame=d['frame']);outputs[case]=a;np.savez_compressed(O/(case+'.npz'),**a)
 (O/'predictions_preheldtruth_seal.json').write_text(json.dumps(dict(utc=now(),artifact_sha256={str(f):sha(f) for f in O.glob('*.npz')},fresh_teacher_not_read_before_seal=True,fit_teacher_sha256={str(H/c/'offline_occupancy_label.npz'):sha(H/c/'offline_occupancy_label.npz') for c in sorted(FIT)},model_type='MODEL reliability, not dynamic action-conditioned worldmodel',metadata=metadata),indent=2)+'\n')
 rows=[]
 for case,a in outputs.items():
  pred=a['probabilities']>=.5;y,truth=labels(case,a['all_voxels']);selected=a['raw_world_voxel_centres'];precision=float(y[pred].mean()) if pred.any() else 0.;xy=truth['origin']+np.argwhere(truth['labels']>0)*float(truth['resolution']);coverage=float((cKDTree(a['possible_xy']).query(xy)[0]<=np.sqrt(2)*.05+1e-9).mean()) if len(a['possible_xy']) else 0.;area=len(a['possible_xy'])*.01;views=class_precision(case,int(a['frame']));minimum=min([v['precision'] for v in views],default=0.);checks=dict(geometry_proxy_precision_ge95=precision>=.95,fixture_coverage_ge95=coverage>=.95,area_le6=area<=6.,birth_class_precision_ge95=bool(views) and minimum>=.95);base=data[case];bp=float(y.mean());bc=float((cKDTree(base['baseline_possible_xy']).query(xy)[0]<=np.sqrt(2)*.05+1e-9).mean());rows.append(dict(case=case,fresh=case not in FIT,rows=len(y),selected_voxels=len(selected),geometry_proxy_precision=precision,fixture_coverage=coverage,area_m2=area,birth_class_precision_min=minimum,checks=checks,baseline=dict(precision=bp,coverage=bc,area_m2=len(base['baseline_possible_xy'])*.01),teacher_sha256=sha(H/case/'offline_occupancy_label.npz'),truth_validity_not_full3D=True,views=views))
 fresh=[r for r in rows if r['fresh']];assert len(fresh)==2, 'both held whole sequences required';passed=all(all(r['checks'].values()) for r in fresh)
 r=dict(utc=now(),decision='PASS_FRESH_MODEL_SOURCE_ONLY_PENDING_CONSUMER' if passed else 'REJECT_UNIQUE_MODEL_RELIABILITY_CANDIDATE',rows=rows,only_one_fit=True,optimizer_iterations=model['iterations'],historical_not_fresh=True,fresh_same_fixture_not_environment_generalization=True,no_actor_action_or_navigation=True,original35_NOTRUN=True,whole_research_complete=False);(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');(O/'final_artifact_seal.json').write_text(json.dumps({str(f):sha(f) for f in O.iterdir() if f.is_file()},indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='rows'},indent=2));print([(v['case'],v['geometry_proxy_precision'],v['fixture_coverage'],v['area_m2']) for v in rows])
if __name__=='__main__':main()
