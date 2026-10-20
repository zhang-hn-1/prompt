"""Postclose phase evidence; no policy replay, proposed actions or counterfactual labels."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 out={};inputs={};fig,ax=plt.subplots(figsize=(8,5))
 for arm in ('b0','candidate'):
  d=H/f'transverse_velocity_feedback_e1_{arm}_run_v1';r=json.loads((d/'policy_records.json').read_text());s=json.loads((d/'states.json').read_text())
  for n in ('policy_records.json','states.json','artifact_manifest_post_close.json','task_episode.json'):inputs[str(d/n)]=sha(d/n)
  groups=[];start=0;previous=None
  for i,x in enumerate(r):
   t=x['tracker_state_after_act'];label=('no_current_route' if x['plan']['status']!='success' else 'direction_braking' if t['pending_direction'] else 'normal_'+t['active_direction']) if x['planner_act_called'] else 'acquisition'
   if label!=previous:
    if previous is not None:groups.append((start,i,previous))
    start=i;previous=label
  groups.append((start,len(r),previous));ep=json.loads((d/'task_episode.json').read_text())['actual_episode'];origin=np.asarray(ep['start_position_world_m'][:2]);goal=np.asarray(ep['goal_position_world_m'][:2]);axis=(goal-origin)/np.linalg.norm(goal-origin);xy=np.asarray([x['position'][:2] for x in s]);cross=(xy-origin)@np.array([-axis[1],axis[0]])
  phases=[]
  for begin,end,label in groups:
   speed=np.array([np.linalg.norm(s[i]['velocity'][:2]) for i in range(begin,end)]);phases.append(dict(start_frame=begin,end_frame_exclusive=end,label=label,duration_s=(end-begin)*.1,actual_net_goal_axis_m=float((xy[end]-xy[begin])@axis),actual_travel_m=float(np.linalg.norm(np.diff(xy[begin:end+1],axis=0),axis=1).sum()),entry_cross_axis_m=float(cross[begin]),exit_cross_axis_m=float(cross[end]),low_XY_sample_count=int((speed<.05).sum()),maximum_logged_stop_count=max(x['tracker_state_after_act']['stopped_cycles'] for x in r[begin:end]),plan_status_counts={k:sum(x.get('plan',{}).get('status','not_planner_call')==k for x in r[begin:end]) for k in set(x.get('plan',{}).get('status','not_planner_call') for x in r[begin:end])}))
  out[arm]=phases;ax.plot(xy[:,0],xy[:,1],label=arm);ax.scatter(xy[-1,0],xy[-1,1],marker='x')
 ax.scatter(*origin,label='start',marker='o');ax.scatter(*goal,label='goal',marker='*');ax.set_aspect('equal');ax.grid();ax.legend();ax.set_xlabel('world X m');ax.set_ylabel('world Y m');ax.set_title('Actual E1 trajectories; frozen initial-goal axis metrics');fig.tight_layout();fig.savefig(H/'transverse_velocity_feedback_e1_trajectory_v1.png',dpi=160);plt.close(fig)
 result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),phases=out,input_sha256=inputs,source_sha256=sha(Path(__file__)),interpretation='Observed phase durations and independent world goal-axis displacement only; zero/request direction and path cursor cannot substitute actual completion. Transverse-velocity feedback is an empirical trial, not a safety certificate or model admission.',no_counterfactual_or_online_claim=True)
 p=H/'transverse_velocity_phase_audit_v1.json';assert not p.exists();p.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
if __name__=='__main__':main()
