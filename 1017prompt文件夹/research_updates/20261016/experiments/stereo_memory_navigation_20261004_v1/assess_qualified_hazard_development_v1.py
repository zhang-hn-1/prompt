from pathlib import Path
import numpy as np,json,hashlib,datetime
H=Path(__file__).resolve().parent;D=H/'qualified_hazard_development_replay_v1';O=H/'observation_handoff_observe_run_recovery_v2';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();seal=json.loads((D/'prediction_seal.json').read_text());assert all(sha(f)==s for f,s in seal['output_sha256'].items());rows=[]
def evaluate(points,uvs,truth):
 errors=[]
 for eye,uv in zip(['left','right'],uvs):
  x,y=np.rint(uv).astype(int).T;z=truth[eye+'_depth_m'][y,x];T=truth[eye+'_camera_usd_world'];R=T[:3,:3]@np.diag([1,-1,-1]);rays=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))]
  with np.errstate(invalid='ignore'):p=T[:3,3]+(rays*z[:,None])@R.T;errors.append(np.linalg.norm(p-points,axis=1))
 e=np.asarray(errors).T;finite=np.isfinite(e).all(1);correct=finite&np.all(e<=.1,axis=1);n=len(points)
 return dict(n=n,correct=int(correct.sum()),insufficient=int((~finite).sum()),finite_wrong=int((finite&~correct).sum()),precision=float(correct.sum()/n) if n else None)
for r in json.loads((D/'records.json').read_text()):
 i=r['frame'];proof=np.load(D/f'proof_{i:03d}.npz');current=np.load(D/f'current_{i:03d}.npz');truth=np.load(O/'offline_truth'/f'frame_{i:03d}.npz');select=proof['appearance_corroborated'];past=evaluate(proof['past_world'][select],[proof['left_uv'][select],proof['right_uv'][select]],truth);h=(current['world'][:,2]>.05)&(current['world'][:,2]<=1.5);q=current['query_pixel_id'][h];p=current['partner_pixel_id'][h];present=evaluate(current['world'][h],[np.c_[q%640,q//640],np.c_[p%640,p//640]],truth);rows.append(dict(frame=i,time_s=r['time_s'],appearance_old=past,qualified_current=present))
valid=[r['appearance_old'] for r in rows if r['appearance_old']['n']];gate=bool(valid) and all(r['precision']>=.95 for r in valid);result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='DEVELOPMENT_OLD_GEOMETRY_PRECISION_PASS' if gate else 'REJECT_DEVELOPMENT_MEMORY_CONSUMER',appearance_precision_ge95_all_nonempty=gate,rows=rows,reused_known_sequence_not_fresh=True,source_only_predictions_sealed_before_labels=True,no_physical_candidate_run_yet=True,old_source_global_REJECT_preserved=True)
f=D/'assessment_v1.json';assert not f.exists();f.write_text(json.dumps(result,indent=2)+'\n');print(result['decision']);print([(r['frame'],r['appearance_old']) for r in rows if r['appearance_old']['n']]);print('qualified_current_aggregate',sum(r['qualified_current']['n'] for r in rows),sum(r['qualified_current']['correct'] for r in rows))
