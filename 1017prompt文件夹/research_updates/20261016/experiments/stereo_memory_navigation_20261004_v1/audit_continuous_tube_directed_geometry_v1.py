from pathlib import Path
import json,numpy as np
H=Path(__file__).resolve().parent;D=H/'E1_continuous_tube_navigation_run_v1';p=json.loads((D/'policy_records.json').read_text());s=json.loads((D/'states.json').read_text());phases=json.loads((D/'physical_phase_audit_v1.json').read_text())['phases'];result=[]
def evaluate(q,ref,edgeids):
 a=ref[:-1];v=np.diff(ref,axis=0);l=np.linalg.norm(v,axis=1);den=(v*v).sum(axis=1);arc=np.r_[0,np.cumsum(l)];u=np.divide(((q-a)*v).sum(axis=1),den,out=np.zeros(len(v)),where=den>0).clip(0,1);dist=np.linalg.norm(q-a-u[:,None]*v,axis=1);j=int(edgeids[np.argmin(dist[edgeids])]);return dict(edge=j,arc_m=float(arc[j]+u[j]*l[j]),distance_m=float(dist[j]))
for phase in phases:
 b,e=phase['begin'],phase['end'];version=phase['state'][0];beg=next(i for i,r in enumerate(p) if r['local_reference_version']==version)
 planpath=D/'local_plans'/f'frame_{beg:03d}.json'
 if not planpath.exists() or phase['state'][3]!='tracking' or phase['state'][4]==0:continue
 wp=json.loads(planpath.read_text())['waypoints'];ref=np.array([[w['x_m'],w['y_m']] for w in wp]);direction=phase['state'][1];ids=np.array([i for i,w in enumerate(wp[:-1]) if w['drive_direction']==direction]);assert len(ids)
 allvals=[evaluate(np.asarray(r['position'][:2]),ref,np.arange(len(ref)-1)) for r in s[b:e+1]];dvals=[evaluate(np.asarray(r['position'][:2]),ref,ids) for r in s[b:e+1]]
 result.append(dict(begin=b,end=e,active_direction=direction,reference_version=version,full_reference_net_progress_m=allvals[-1]['arc_m']-allvals[0]['arc_m'],same_declared_direction_net_progress_m=dvals[-1]['arc_m']-dvals[0]['arc_m'],same_direction_peak_error_m=max(x['distance_m'] for x in dvals),same_direction_enter_error_m=dvals[0]['distance_m'],same_direction_exit_error_m=dvals[-1]['distance_m'],full_vs_directed_projection_disagreement_samples=sum(a['edge']!=z['edge'] for a,z in zip(allvals,dvals)),actual_state_start=s[b],actual_state_end=s[e]))
 out=None
out=dict(scope='POSTCLOSE development two independent frozen geometry evaluators. Uses actual phase direction and saved waypoint outgoing drive_direction to restrict edges. No policy path_index. Reversing folded references make global nearest arclength ambiguous; directed projection is diagnostic, not completed-segment or navigation success. No new gate/control/source.',phases=result,formal35_NOTRUN=True)
(D/'directed_geometry_audit_v1.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps([{k:v for k,v in r.items() if not k.startswith('actual_state')} for r in result]))
