from pathlib import Path
import json,hashlib,datetime,numpy as np
from dynamic_self_mask import RobotSelfMask,joints_from_proprio
from stereo_geometry import FOCAL
H=Path(__file__).resolve().parent;D=H/'native_fixed_appearance_probe_run_v1';O=H/'fixed_appearance_raw_memory_visibility_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();cad=RobotSelfMask();rows=[];inputs={}
manifest=json.loads((D/'artifact_manifest_post_close.json').read_text())['artifact_sha256']
for f,s in manifest.items():assert sha(f)==s,f
for row in json.loads((H/'fixed_appearance_raw_point_memory_v1/memory_shadow_rows.json').read_text()):
 i=row['frame'];name=f'frame_{i:03d}';files=[H/'fixed_appearance_raw_point_memory_v1/tentative_memory_snapshots'/(name+'.json'),D/'estimated_poses'/(name+'.npz'),D/'history'/(name+'.npz'),D/'partial_class_masks'/(name+'.npz')]
 for f in files:inputs[str(f)]=sha(f)
 memory=json.loads(files[0].read_text());pose=np.load(files[1]);history=np.load(files[2]);classes=np.load(files[3]);masks=cad.project(joints_from_proprio(history['proprio_history']));now=row['time_s'];past=[r for r in memory['records'] if r['time_s']<now];p=np.concatenate([np.asarray(r['world']).reshape(-1,3) for r in past],axis=0) if past else np.empty((0,3));assert len(p)==row['historical_count'];eyes={};visible_masks=[]
 for eye in ('left','right'):
  T=pose[eye+'_camera_usd_world'];R=T[:3,:3]@np.diag([1,-1,-1]);local=(p-T[:3,3])@R;z=local[:,2];front=z>0;uv=np.full((len(p),2),-1.)
  uv[front]=local[front,:2]/z[front,None]*FOCAL+[319.5,239.5];inside=front&(uv[:,0]>=0)&(uv[:,0]<640)&(uv[:,1]>=0)&(uv[:,1]<480);xy=np.floor(uv[inside]).astype(int);body=np.zeros(len(p),bool);positive=np.zeros(len(p),bool);body[inside]=masks[eye][xy[:,1],xy[:,0]];positive[inside]=classes[eye][xy[:,1],xy[:,0]];visible=inside&~body;visible_masks.append(visible)
  eyes[eye]=dict(total=len(p),behind=int((~front).sum()),outside=int((front&~inside).sum()),bodyblocked=int((inside&body).sum()),geometrically_visible=int(visible.sum()),current_positive_diagnostic=int((visible&positive).sum()))
 rows.append(dict(frame=i,time_s=now,historical_records=len(p),eyes=eyes,hidden_at_least_one_eye=int((~(visible_masks[0]&visible_masks[1])).sum()),hidden_both_eyes=int((~(visible_masks[0]|visible_masks[1])).sum()),old_source_times_preserved=True,no_new_observed_or_free_label=True))
(O/'visibility_rows.json').write_text(json.dumps(rows,indent=2)+'\n');(O/'seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),inputs=inputs,code_sha256=sha(__file__),rows_sha256=sha(O/'visibility_rows.json'),no_truth_label_reads=True),indent=2)+'\n');a=dict(all29epochs=len(rows),historical_record_epoch_pairs=sum(r['historical_records'] for r in rows),hidden_at_least_one_eye_pairs=sum(r['hidden_at_least_one_eye'] for r in rows),hidden_both_eye_pairs=sum(r['hidden_both_eyes'] for r in rows),final=rows[-1],not_independent_points_or_3D_volume=True,no_navigation_or_source_admission=True,whole_research_complete=False);(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps(a,indent=2))
