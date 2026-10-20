from pathlib import Path
import json,hashlib,sys
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from baseline35_metrics_v1 import assess,observation_gap_diagnostics
H=Path(__file__).resolve().parent
status=json.loads((H/'baseline35_status_v1.json').read_text());assert status['status']=='finished' and status['exit_code']==0
results=[]
for name in ('e0','e2'):
 D=H/f'baseline35_{name}_run_v1';manifest=json.loads((D/'artifact_manifest_post_close.json').read_text());assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==d for p,d in manifest['artifact_sha256'].items())
 s=json.loads((D/'states.json').read_text());c=json.loads((D/'physics_contacts.json').read_text());ep=json.loads((D/'task_episode.json').read_text())['actual_episode'];req=np.load(D/'requests.npz')['requests'];physics=np.load(D/'physics.npz');termination=json.loads((D/'termination.json').read_text());assert len(req)<=350 and len(s)==len(req)+1;assert len(physics['simulation_time_s'])==len(req)*5+1;assert max(a['time_s'] for a in s)<=35+1e-5
 r=assess(s,[dict(time_s=a['simulation_time_s'],collision=a['contact'],available=a['available']) for a in c],ep['start_position_world_m'][:2],ep['goal_position_world_m'][:2]);r.update(episode_id=ep['episode_id'],scope='NEW_ACTUAL_KNOWNPOSE_TASK_PRIOR_BASELINE35_NOT_PURE_VISUAL',requests=len(req),captures=len(s),physics_samples=len(physics['simulation_time_s']),runtime_termination=termination,raw_manifest_verified=True,source_accuracy_is_diagnostic_only=True,controller_unchanged=True,no_model_fit=True,not_fresh_for_method_design=True)
 videos={}
 for n in ('stereo_view','third_person'):
  cap=cv2.VideoCapture(str(D/(n+'.mp4')));fps=cap.get(cv2.CAP_PROP_FPS);count=0
  while True:
   ok,img=cap.read()
   if not ok:break
   if count in (0,len(s)//2,len(s)-1):cv2.imwrite(str(D/f'{n}_frame_{count:03d}.png'),img)
   count+=1
  cap.release();assert count==len(s) and abs(fps-10)<1e-6;videos[n]=dict(decoded_frames=count,fps=fps)
 r['videos']=videos;rows=json.loads((D/'policy_records.json').read_text());assert len(rows)==len(req);r['observation_gaps']=observation_gap_diagnostics(json.loads((D/'source_rows.json').read_text()),rows);lat=[a['end_to_end_wall_s'] for a in rows];r['cycle_wall_s']=dict(median=float(np.median(lat)),p90=float(np.quantile(lat,.9)),maximum=float(max(lat)),fraction_over_0p1=float(np.mean(np.asarray(lat)>.1)),realtime_10Hz=all(a<=.1 for a in lat)) if lat else None
 xy=np.array([a['position'][:2] for a in s]);start=np.asarray(ep['start_position_world_m'][:2]);goal=np.asarray(ep['goal_position_world_m'][:2]);fig,axs=plt.subplots(1,2,figsize=(10,4));axs[0].plot(xy[:,0],xy[:,1]);axs[0].scatter([start[0],goal[0]],[start[1],goal[1]],c=['green','red']);axs[0].set_aspect('equal');axs[0].set_title(ep['episode_id']+' actual XY');axs[1].plot([a['time_s'] for a in s],np.linalg.norm(xy-goal,axis=1));axs[1].set_xlabel('simulation seconds');axs[1].set_ylabel('goal distance m');fig.tight_layout();fig.savefig(D/'trajectory_and_goal_distance_v1.png',dpi=150);plt.close(fig)
 (D/'baseline35_metrics_v1.json').write_text(json.dumps(r,indent=2)+'\n');results.append(r)
e1=json.loads((H/'baseline35_e1_historical_metrics_v1.json').read_text());allr=[results[0],e1,results[1]];(H/'baseline35_combined_metrics_v1.json').write_text(json.dumps(dict(results=allr,not_a_three_fresh_episode_success_rate=True,no_causal_AB=True,no_strict_visual_admission=True,no_high_source_accuracy_gate=True),indent=2)+'\n');print([(r['episode_id'],r['outcome'],r['goal_remaining_m']) for r in allr])
