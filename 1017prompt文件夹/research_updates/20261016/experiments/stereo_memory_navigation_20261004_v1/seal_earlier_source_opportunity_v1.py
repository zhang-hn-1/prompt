"""Select first causal consecutive source pair BEFORE labels; historical development."""
from pathlib import Path
import json,hashlib,datetime,numpy as np
from earlier_source_opportunity import EarlierSourceOpportunity
H=Path(__file__).resolve().parent;D=H/'qualified_hazard_navigation_run_v1';O=H/'earlier_source_opportunity_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();manifest=json.loads((D/'artifact_manifest_post_close.json').read_text());sp=D/'states.json';assert sha(sp)==manifest['artifact_sha256'][str(sp)];states=json.loads(sp.read_text());a=EarlierSourceOpportunity();prefix=[];paths=[Path(__file__),H/'earlier_source_opportunity.py',H/'test_earlier_source_opportunity.py',H.parent.parent/'docs/superpowers/plans/2026-10-07-earlier-qualified-source-opportunity.md',sp];pair=None
for i in range(0,len(states),5):
 p=D/'sensor_packets'/f'source_{i:03d}.npz'
 if not p.exists():break
 assert sha(p)==manifest['artifact_sha256'][str(p)];paths.append(p);f=np.load(p);t=float(states[i]['time_s']);v=np.array([r['velocity'][:2] for r in states[max(0,i-4):i+1]]);pair=a.update(f,v,t,i);prefix.append(dict(a.diagnostics))
 if pair:break
assert pair is not None,'No early source opportunity'
for i in pair:
 f=np.load(D/'sensor_packets'/f'source_{i:03d}.npz');sel=(f['world'][:,2]>.05)&(f['world'][:,2]<=1.5);np.savez_compressed(O/f'source_{i:03d}.npz',world=f['world'][sel],query_pixel_id=f['query_pixel_id'][sel],partner_pixel_id=f['partner_pixel_id'][sel],frame=i,time_s=f['time_s'],source_model_admitted=False,certified_free=False)
(O/'records.json').write_text(json.dumps(dict(pair=pair,event_frame=pair[-1],event_time_s=float(states[pair[-1]]['time_s']),prefix=prefix,selected_before_labels=True,source_only_not_actual_control=True,knownpose_privileged=True,old18window_unchanged=True,fresh_heldout=False),indent=2)+'\n');(O/'source_custody.json').write_text(json.dumps(dict(source_sha256={str(p):sha(p) for p in paths},GT_read=False,policy_calls=0,NN_calls=0,physics_calls=0),indent=2)+'\n');(O/'prediction_seal.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),output_sha256={str(p):sha(p) for p in O.iterdir() if p.is_file()},no_GT_before_selection=True,not_whole_research_complete=True),indent=2)+'\n');print(pair)
