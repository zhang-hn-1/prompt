"""Birth-label quality of sealed historical sparse memory, no actor permission."""
from pathlib import Path
import json,hashlib,datetime,numpy as np
from dynamic_self_mask import RobotSelfMask,joints_from_proprio
from stereo_geometry import FOCAL
H=Path(__file__).resolve().parent;D=H/'native_flat_pose_navigation_R1_run_v1';M=H/'whole_historical_moving_memory_v1';O=H/'whole_historical_memory_quality_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((M/'actor_seal.json').read_text())
for group in ['input_sha256','output_sha256']:
 for f,s in seal[group].items():assert sha(f)==s,f
assert seal['no_truth_label_reads'] and seal['offline_historical_not_online']
O.mkdir(exist_ok=False);rows=json.loads((M/'rows.json').read_text());cad=RobotSelfMask();birth={};errors=[];classes=[];source_epochs=[];labelhash={}
def metric(e):
 e=np.asarray(e,float)
 return dict(n=len(e),within_0p10m=float(np.mean(e<=.1)) if len(e) else None,unconditional_p90_m=float(np.sort(e)[int(np.ceil(.9*len(e)))-1]) if len(e) else None,invalid_count=int((~np.isfinite(e)).sum()))
for row in rows:
 i=row['frame'];name=f'frame_{i:03d}.npz';p=np.load(M/'points'/name);gtfile=D/'offline_truth'/name;labelhash[str(gtfile)]=sha(gtfile);gt=np.load(gtfile);q=p['query_pixel_id'];y,x=np.divmod(q,640);z=gt['left_depth_m'][y,x];rays=np.column_stack([(x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(len(q))]);T=gt['left_camera_usd_world'];truth=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;valid=np.isfinite(truth).all(1)&np.isfinite(z)&(z>0);e=np.linalg.norm(p['world']-truth,axis=1);e[~valid]=np.inf;c=valid&(np.linalg.norm(truth[:,:2],axis=1)<=.351)&(truth[:,2]>.001)&(truth[:,2]<=1.001)
 errors.extend(e.tolist());classes.extend(c.tolist());source_epochs.append(dict(frame=i,time_s=row['time_s'],metric=metric(e),class_precision=float(c.mean()) if len(c) else None))
 birth[i]=dict(world=p['world'].copy(),query=q.copy(),partner=p['partner_pixel_id'].copy(),error=e.copy(),time=float(p['time_s']))
 current=[];past=[];one=[];both=[];epoch=[];verified=0
for row in rows:
 i=row['frame'];name=f'frame_{i:03d}.npz';memory=json.loads((M/'memory'/f'frame_{i:03d}.json').read_text());pose=np.load(D/'estimated_poses'/name);history=np.load(D/'history'/name);masks=cad.project(joints_from_proprio(history['proprio_history']));pastworld=[];paste=[];cur=[]
 for r in memory['records']:
  b=birth[r['frame']];np.testing.assert_array_equal(r['world'],b['world']);np.testing.assert_array_equal(r['query_pixel_id'],b['query']);np.testing.assert_array_equal(r['partner_pixel_id'],b['partner']);assert r['time_s']==b['time']<=row['time_s'];assert r['layer']=='TENTATIVE_RGB_DEPTH_SURFACE_SOURCE_REJECTED_DIAGNOSTIC';verified+=len(b['error'])
  if b['time']==row['time_s']:cur.extend(b['error'].tolist())
  else:pastworld.extend(b['world'].tolist());paste.extend(b['error'].tolist())
 pe=np.asarray(paste,float);world=np.asarray(pastworld,float).reshape(-1,3);visible=[]
 for eye in ['left','right']:
  T=pose[eye+'_camera_usd_world'];local=(world-T[:3,3])@(T[:3,:3]@np.diag([1,-1,-1]));z=local[:,2];front=z>0;uv=np.full((len(world),2),-1.);uv[front]=local[front,:2]/z[front,None]*FOCAL+[319.5,239.5];inside=front&(uv[:,0]>=0)&(uv[:,0]<640)&(uv[:,1]>=0)&(uv[:,1]<480);px=np.floor(uv[inside]).astype(int);blocked=np.zeros(len(world),bool);blocked[inside]=masks[eye][px[:,1],px[:,0]];visible.append(inside&~blocked)
 hidden1=~(visible[0]&visible[1]);hidden2=~(visible[0]|visible[1]);assert int(hidden1.sum())==row['hidden_one_or_both'] and int(hidden2.sum())==row['hidden_both'];assert len(cur)==row['current_count'] and len(pe)==row['historical_count']
 current.extend(cur);past.extend(pe.tolist());one.extend(pe[hidden1].tolist());both.extend(pe[hidden2].tolist());epoch.append(dict(frame=i,time_s=row['time_s'],current=metric(cur),past=metric(pe),hidden_at_least_one_eye=metric(pe[hidden1]),hidden_both=metric(pe[hidden2]),distinct_XY_cells=row['distinct_XY_cells']))
e=metric(errors);gates=dict(n100=e['n']>=100,metric95=e['within_0p10m']>=.95,p90=e['unconditional_p90_m']<=.1,class95=float(np.mean(classes))>=.95,two100_exposures=sum(r['metric']['n']>=100 for r in source_epochs)>=2)
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_HISTORICAL_POINT_METRIC_ONLY' if all(gates.values()) else 'REJECT_HISTORICAL_SPARSE_SOURCE_ADMISSION',gates=gates,unique_birth_records=e,obstacle_class_precision=float(np.mean(classes)),current_record_epoch=metric(current),historical_record_epoch=metric(past),historical_hidden_at_least_one_eye_record_epoch=metric(one),historical_hidden_both_record_epoch=metric(both),immutable_provenance_records_verified=verified,peak_distinct_XY_cells=max(r['distinct_XY_cells'] for r in rows),epochs=len(rows),source_epochs=source_epochs,memory_epochs=epoch,birth_labels_only_not_future_surface_labels=True,repeated_record_epochs_not_independent_samples=True,history_retention_does_not_increase_birth_point_count=True,no_new_volume_coverage_or_free_space_certificate=True,no_source_gate_lowered=True,no_NN_or_runtime_or_new_actor=True,old_rejects_preserved=True,whole_research_complete=False,old_actor_seal_sha256=sha(M/'actor_seal.json'),offline_label_sha256=labelhash,code_sha256=sha(__file__))
(O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ['source_epochs','memory_epochs','offline_label_sha256']},indent=2))
