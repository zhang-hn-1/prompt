"""Bounded historical risk-age diagnostic, not a controller or truth certificate."""
from pathlib import Path
import json,hashlib,datetime,numpy as np
H=Path(__file__).resolve().parent;O=H/'baseline_persistent_risk_age_audit_v1';O.mkdir(exist_ok=False);runs=['native_pose_baseline35_e1_coverage_run_v1','native_pose_baseline35_e2_coverage_run_v1'];protocol=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),runs=runs,scope='historical maps only, before run scoring end; native executed baseline',query='persistent positive-cell acquisition age exceeding existing raw record TTL10s',threshold_not_scanned=10.,no_teacher_or_new_source_read=True,no_actor_change=True,no_unknown_as_free=True,not_a_bug_claim=True);(O/'protocol_before_analysis.json').write_text(json.dumps(protocol,indent=2)+'\n');rows=[]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for name in runs:
 D=H/name;seal=json.loads((D/'artifact_manifest_post_close.json').read_text());assert all(sha(p)==h for p,h in seal['artifact_sha256'].items());m=json.loads((D/'metrics.json').read_text());cut=float(m['elapsed_s']);timeline=[]
 for f in sorted((D/'maps').glob('frame_*.npz')):
  a=np.load(f);t=float(a['time_s'])
  if t>cut+1e-6:continue
  active=a['labels']==1;clock=a['hazard_acquisition_time'];assert np.all(clock[active]<=t+1e-6);old=active&((t-clock)>10.)
  timeline.append(dict(frame=int(f.stem.split('_')[-1]),time_s=t,positive_cells=int(active.sum()),positive_cells_older_than_raw_TTL=int(old.sum()),unknown_cells=int((a['labels']==-1).sum())))
 rows.append(dict(run=name,cut_s=cut,frames=len(timeline),frames_with_stale_positive=sum(x['positive_cells_older_than_raw_TTL']>0 for x in timeline),peak_stale_positive=max(x['positive_cells_older_than_raw_TTL'] for x in timeline),timeline=timeline));print({k:v for k,v in rows[-1].items() if k!='timeline'},flush=True)
(O/'assessment.json').write_text(json.dumps(dict(protocol=protocol,rows=rows,interpretation='static scene persistence is intended, but source uncertainty is not independently cured by static world; older cells may be real or false, age alone is not evidence of free space',no_new_candidate_acceptance=True),indent=2)+'\n');(O/'artifact_manifest_post_close.json').write_text(json.dumps(dict(artifact_sha256={str(p):sha(p) for p in O.rglob('*') if p.is_file()}),indent=2)+'\n')
