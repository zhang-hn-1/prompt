"""One bounded sealed-log attribution; no planner/control/map mutation or replay."""
from paths import *
import json,hashlib,numpy as np
from collections import Counter
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
cfg=json.loads((D/'trace_protocol.json').read_text())
for p,h in cfg['input_sha256'].items():assert sha(p)==h,p
B=W/'experiments/four_camera_weak_baseline_20261017_v1';arms={}
for name,root in [('A',D),('B',B)]:
    rows=json.loads((root/'e1_run/policy_records.json').read_text());assert len(rows)==900
    decisions=Counter();rejects=Counter();versions=[];events=[];lastversion=None;switches=[];lastsign=None
    for i,r in enumerate(rows):
        rd=r.get('reference_decision') or {};decisions[rd.get('decision','none')]+=1
        guard=rd.get('guard') or {};version=int(r['local_reference_version']);new=version!=lastversion
        if new:
            versions.append(i);lastversion=version
            if rd.get('decision')=='rebuild':
                for reason in guard.get('reject_reasons',[]):rejects[str(reason)]+=1
                events.append(dict(policy=i,time_s=r['time_s'],version=version,initial_or_no_route=bool(rd.get('initial_or_no_route')),guard_reject_reasons=guard.get('reject_reasons',[]),guard_retention_screen_pass=guard.get('retention_necessary_screen_pass'),source_frame=r['source_frame'],source_age_s=r['source_age_s'],previous_reference_age_s=rows[i-1].get('reference_age_s') if i else None,pending_before=(rd.get('state_before') or {}).get('pending_direction'),active_before=(rd.get('state_before') or {}).get('active_direction')))
        s=int(np.sign(np.asarray(r['request'])[::2].mean()))
        if s:
            if lastsign is not None and s!=lastsign:
                recent=[e for e in events if 0<=r['time_s']-e['time_s']<=1.000001]
                switches.append(dict(policy=i,time_s=r['time_s'],from_sign=lastsign,to_sign=s,rebuild_within1s=bool(recent),source_refreshed=r['source_refreshed'],pending_direction=r['tracker_state']['pending_direction']))
            lastsign=s
    spans=[]
    for k,start in enumerate(versions):
        end=versions[k+1] if k+1<len(versions) else len(rows)
        xyz=np.array([r['estimated_body'] for r in rows[start:end]])[:,:2,3]
        spans.append(dict(version=rows[start]['local_reference_version'],policy_start=start,policy_end_exclusive=end,duration_s=(end-start)*.1,estimated_path_m=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum()) if len(xyz)>1 else 0.,estimated_displacement_m=float(np.linalg.norm(xyz[-1]-xyz[0])) if len(xyz)>1 else 0.))
    noninitial=[e for e in events if not e['initial_or_no_route']];guardpass=[e for e in noninitial if e['guard_retention_screen_pass'] is True];pending=sum(r['tracker_state']['pending_direction'] is not None for r in rows)
    arms[name]=dict(reference_decision_policy_counts=dict(decisions),new_version_count=len(versions),rebuild_new_version_events=len(events),noninitial_rebuild_events=len(noninitial),noninitial_rebuild_guard_pass_count=len(guardpass),guard_reject_reason_counts=dict(rejects),request_direction_switches=len(switches),request_switch_within1s_after_rebuild=sum(s['rebuild_within1s'] for s in switches),pending_direction_policy_queries=pending,pending_direction_s=pending*.1,version_spans=spans,events=events,switches=switches)
# A necessary rejection of the simple "unguarded replacement" claim, not a global no-planning-problem proof.
result=dict(complete=True,rows=1800,arms=arms,unguarded_rebuild_example_present=any(a['noninitial_rebuild_guard_pass_count']>0 for a in arms.values()),scope='Complete sealed A/B policy-log facts. Repeated retained decisions are not new references. Near-time switch/rebuild association is not causation. Guardfail may reflect wrong perception or dynamics; not independently certified obstacle/noise. No new primitive/controller/reference rule admitted.',fit=0,physics=0,replay=0,source_regeneration=0,protocol_sha256=sha(D/'trace_protocol.json'))
(D/'trace_result.json').write_text(json.dumps(result,indent=2)+'\n');print('TRACE',{n:{k:v for k,v in a.items() if k not in ['version_spans','events','switches']} for n,a in arms.items()},flush=True)
