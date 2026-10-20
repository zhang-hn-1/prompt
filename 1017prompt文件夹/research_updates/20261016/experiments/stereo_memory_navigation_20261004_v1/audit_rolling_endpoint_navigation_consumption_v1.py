"""One bounded post-run phase attribution; no new controller or counterfactual rollout."""
from pathlib import Path
from collections import Counter
import json,hashlib,numpy as np
H=Path(__file__).parent
result=json.loads((H/'rolling_endpoint_navigation_e0_assessment_v1.json').read_text());out={}
for arm in ('b0','paced'):
 D=H/('rolling_endpoint_navigation_e0_'+arm+'_run_v1');ss=json.loads((D/'states.json').read_text());rows=json.loads((D/'policy_records.json').read_text());e=json.loads((D/'task_episode.json').read_text())['actual_episode'];start=np.array(e['start_position_world_m'][:2]);goal=np.array(e['goal_position_world_m'][:2]);axis=(goal-start)/np.linalg.norm(goal-start);end=result['arms'][arm]['scored_end_s'];rr=[r for r in rows if r['time_s']+.1<=end+1e-5];groups={};reasons=Counter()
 for r in rr:
  version=str(r.get('local_reference_version',0));f=r['frame'];g=groups.setdefault(version,dict(first_frame=f,last_frame=f,intervals=0,actual_axis_gain_m=0.,path_length_m=0.,main_request_min=10.,main_request_max=-10.,actual_speed_mean_sum=0.,interventions=0,reserve_restored=0));g['last_frame']=f;g['intervals']+=1;delta=np.array(ss[f+1]['position'][:2])-np.array(ss[f]['position'][:2]);g['actual_axis_gain_m']+=float(delta@axis);g['path_length_m']+=float(np.linalg.norm(delta));g['actual_speed_mean_sum']+=float(np.linalg.norm(ss[f]['velocity'][:2]));m=np.array(r['request'])[::2];g['main_request_min']=min(g['main_request_min'],float(m.min()));g['main_request_max']=max(g['main_request_max'],float(m.max()));g['interventions']+=int(r.get('rolling_control_applied',False));g['reserve_restored']+=int('preserve_original_within_same_reserve' in r.get('measured_speed_control_reasons',[]));reasons.update(r.get('measured_speed_control_reasons',[]))
 for g in groups.values():g['actual_speed_mean_m_s']=g.pop('actual_speed_mean_sum')/g['intervals'];g['duration_s']=.1*g['intervals']
 out[arm]=dict(phases_by_reference_version=groups,eligibility_reasons_overlapping=dict(reasons),scored_intervals=len(rr),post_contact_intervals_excluded=True)
p=H/'rolling_endpoint_navigation_consumption_audit_v1.json';assert not p.exists();p.write_text(json.dumps(dict(arms=out,scope='Actual completed intervals attribution only; cannot assert another target, reserve or plan would succeed',new_physics=0,new_fit=0,whole_mission_complete=False,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2)+'\n');print(json.dumps(out),flush=True)
