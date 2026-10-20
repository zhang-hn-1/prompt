"""Postclose reconstruct actual event consumer and originalhandoff beforelabels."""
from pathlib import Path
import datetime,hashlib,json
import numpy as np
from observation_handoff import ObservationHandoff
from neutral_bank_visibility_probe import NeutralBankVisibilityProbe
H=Path(__file__).resolve().parent;D=H/'neutral_bank_visibility_run_v1'
status=json.loads((H/'neutral_bank_visibility_status_v1.json').read_text())
assert status['stage']=='finished_actual_probe_only' and status['exit_code']==0
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((D/'artifact_manifest_post_close.json').read_text())
assert all(sha(p)==v for p,v in seal['artifact_sha256'].items())
rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text())
g=ObservationHandoff();p=NeutralBankVisibilityProbe();fields=0;effective=0;active=[]
def one(g,p,i,row):
    local=row['local'];t=float(local['time_s']);sf=int(local['source_frame'])
    assert sf<=i and float(local['source_time_s'])<=t
    source=np.load(D/'sensor_packets'/f'source_{sf:03d}.npz')
    qualified=np.load(D/'qualified_sensor_packets'/f'source_{sf:03d}.npz')
    xyz=qualified['world'];n=int(((xyz[:,2]>.05)&(xyz[:,2]<=1.5)).sum())
    request=g.choose(t,float(np.linalg.norm(states[i]['velocity'][:2])),source,np.array(local['nominal_request']))
    np.testing.assert_array_equal(request,local['pre_probe_request'])
    out=p.choose(t,sf,float(source['time_s']),n,g.handed_off,request)
    np.testing.assert_array_equal(out,local['request'])
    # Top-level request is inherited nominal diagnostic dictionary, NOT final10D request.
    np.testing.assert_array_equal(out,np.load(D/'policy_inputs'/f'frame_{i:03d}.npz')['request'])
    assert p.diagnostics==local['visibility_probe']
    assert g.handed_off==local['observation_handed_off'] and g.low_count==local['observation_low_count']
    assert local['probe_effective_request_changed']==bool(np.any(out!=request))
    return out.copy()
outputs=[]
for i,r in enumerate(rows):
    outputs.append(one(g,p,i,r));fields+=len(p.diagnostics)
    if r['local']['visibility_probe']['active']:active.append(i)
    effective+=int(r['local']['probe_effective_request_changed'])
stop=(active[-1]+2) if active else min(len(rows),100)
stop=min(stop,len(rows));pg=ObservationHandoff();pp=NeutralBankVisibilityProbe()
for i,r in enumerate(rows[:stop]):np.testing.assert_array_equal(one(pg,pp,i,r),outputs[i])
future_traps=0
class Trap:
    def __array__(self,*a,**kw):raise AssertionError('future request payload read')
for i in range(min(stop,10)):
    q=NeutralBankVisibilityProbe()
    try:q.choose(float(i),i,float(i)+1,0,True,Trap())
    except ValueError:future_traps+=1
    else:raise AssertionError('future source accepted')
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),requests=len(rows),probe_diagnostic_fields=fields,
            originalhandoff_reconstruction_exact=True,actualfinal_request_reconstruction_exact=True,
            event_time_s=p.event_time,event_frame=p.event_frame,active_request_count=len(active),
            active_first_frame=active[0] if active else None,active_last_frame=active[-1] if active else None,
            changed_final_requests=effective,prefix_only_request_count=stop,prefix_outputs_exact=True,
            future_action_payload_traps=future_traps,renderer_labels_read=False,
            no_archive_wide_future_log_poison_claim=True,source_prefix_vs_historical_B0_not_certified=True)
out=H/'neutral_bank_visibility_interface_verification_v2.json';assert not out.exists()
out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
