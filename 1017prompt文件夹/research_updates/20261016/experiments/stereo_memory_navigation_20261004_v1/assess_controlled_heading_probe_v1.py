"""ONE source-only sequence; postclose precision, view and execution gates."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np,cv2
H=Path(__file__).resolve().parent;D=H/'controlled_heading_probe_run_v1';status=json.loads((H/'controlled_heading_probe_stage_status_v1.json').read_text());assert status['stage']=='ACTUAL_CLOSED_NEEDS_VERIFICATION';sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();seal=D/'artifact_manifest_post_close.json';assert sha(seal)==status['post_close_manifest_sha256'];hashes=json.loads(seal.read_text())['artifact_sha256'];assert all(sha(f)==s for f,s in hashes.items());causal=json.loads((H/'controlled_heading_probe_causal_verification_v1.json').read_text());visible=json.loads((H/'controlled_heading_probe_visibility_v1/assessment_v1.json').read_text())
def geometry(d,p):
 w=p['world'];haz=(w[:,2]>.05)&(w[:,2]<=1.5);w=w[haz];i=int(p['frame']);gt=np.load(d/'offline_truth'/f'frame_{i:03d}.npz');es=[]
 for eye,key in [('left','query_pixel_id'),('right','partner_pixel_id')]:
  idx=p[key][haz];x,y=idx%640,idx//640;z=gt[eye+'_depth_m'][y,x];T=gt[eye+'_camera_usd_world'];ray=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))]
  with np.errstate(invalid='ignore'):actual=T[:3,3]+(ray*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;es.append(np.linalg.norm(actual-w,axis=1))
 e=np.asarray(es).T;finite=np.isfinite(e).all(1);ok=finite&(e<=.1).all(1)
 return dict(n=len(w),correct=int(ok.sum()),insufficient=int((~finite).sum()),finite_wrong=int((finite&~ok).sum()),precision=float(ok.mean()) if len(w) else None,p90_finite_max_endpoint_error_m=float(np.quantile(e[finite].max(1),.9)) if finite.any() else None,correct_XY_bins=len(np.unique(np.floor(w[ok,:2]/.1).astype(int),axis=0)))


scores=[]
for f in sorted((D/'sensor_packets').glob('source_*.npz')):
 p=np.load(f);filtered=np.load(D/'qualified_sensor_packets'/f.name);scores.append(dict(frame=int(p['frame']),time_s=float(p['time_s']),raw=geometry(D,p),qualified=geometry(D,filtered)))
opp=json.loads((D/'source_opportunity_records.json').read_text());pair=next((r['source_event']['pair'] for r in opp if r['source_event']),None);used=[r for r in scores if pair and r['frame']>=pair[1]];pairrows=[r for r in scores if pair and r['frame'] in pair];raw_pass=bool(pairrows) and all(r['raw']['n']>=100 and r['raw']['precision']>=.95 and r['raw']['correct_XY_bins']>=2 for r in pairrows);q=[r['qualified'] for r in used if r['qualified']['n']];filtered_pass=bool(q) and all(r['precision']>=.95 for r in q)
views=[]
for r in visible['rows']:
 left=next(e for e in r['eyes'] if e['eye']=='left');views.append(dict(frame=r['frame'],common_pixels=left['mutually_unmasked_same_surface']))
viewpass=any(a['common_pixels']>=100 and b['common_pixels']>=100 and b['frame']-a['frame']==5 for a,b in zip(views[:-1],views[1:]))
rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());clocks=json.loads((D/'camera_clocks.json').read_text());N=len(rows);ph=np.load(D/'physics.npz');assert N==80 and len(states)==len(clocks)==81 and len(ph['simulation_time_s'])==401
assert np.array_equal(np.array([r['local']['request'] for r in rows]),np.zeros((80,10))) and np.max(np.abs(ph['applied_action'][:,0]))==0
assert all(not r['local']['observation_handed_off'] and r['local']['observation_handoff_time_s'] is None for r in rows)
assert max(abs(r['raw_time_s']-et) for r in clocks for et in r['eyes'].values())<=1e-5
videos=[]
for name in ['stereo_view','third_person']:
 cap=cv2.VideoCapture(str(D/(name+'.mp4')));n=0
 while True:
  ok,_=cap.read()
  if not ok:break
  n+=1
 fps=cap.get(cv2.CAP_PROP_FPS);cap.release();assert n==81 and fps==10.;videos.append(dict(name=name,frames=n,fps=fps))
xy=np.array([s['position'][:2] for s in states]);rootdis=float(np.linalg.norm(xy[-1]-xy[0]));contacts=sum(bool(c['contact']) for c in json.loads((D/'physics_contacts.json').read_text()));passall=bool(viewpass and raw_pass and filtered_pass and not contacts and rootdis<=.7)
out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_LOCAL_SOURCE_PREREQUISITE_ONLY' if passall else 'REJECT_HEADING_ONLY_SOURCE_PREREQUISITE',actual_source_event_pair=pair,mutual_visibility_two_consecutive_ge100=viewpass,raw_pair_precision_ge95_and_bins2=raw_pass,qualified_all_nonempty_precision_ge95=filtered_pass,raw_and_qualified_scores=scores,view_rows=views,source_only=True,navigation_executed=False,memory_candidate_executed=False,source_model_global_admitted=False,original_yaw_minus20_failure_preserved=True,formal_triggered=False,zero_requests_verified=True,body_root_displacement_m=rootdis,path_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),contacts=contacts,captures=81,source_calls=len(scores),videos=videos,physical_clone=causal['arms'][0],whole_research_complete=False,GT_only_postclose=True,visibility_XY_bins_gate_enforced_by_qualifying_raw_source=True)
f=H/'controlled_heading_probe_assessment_v1.json';assert not f.exists();f.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:v for k,v in out.items() if k not in ['raw_and_qualified_scores','physical_clone']},indent=2))
