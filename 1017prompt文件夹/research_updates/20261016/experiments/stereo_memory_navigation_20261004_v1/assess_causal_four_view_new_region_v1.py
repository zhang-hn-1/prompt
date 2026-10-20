"""Post-seal labels only. Frozen groupwise precision and spatial-coverage gates."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
from scipy.spatial import cKDTree
H=Path(__file__).resolve().parent;D=H/'causal_four_view_new_region_v1';O=H/'qualified_hazard_navigation_run_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((D/'prediction_seal.json').read_text())
for kind in ['input_sha256','output_sha256']:
 for p,h in seal[kind].items():assert sha(p)==h,p
seed=np.load(O/'sensor_packets'/'source_180.npz')
a=seed['world'];a=a[(a[:,2]>.05)&(a[:,2]<=1.5)];initial_bins=set(map(tuple,np.floor(a[:,:2]/.1).astype(int)));tree=cKDTree(a[:,:2]);states=json.loads((O/'states.json').read_text());rows=[];allgroups=[]
def score(points,uvs,truth):
 err=[]
 for eye,uv in zip(['left','right'],uvs):
  x,y=np.rint(uv).astype(int).T;assert np.all((x>=0)&(x<640)&(y>=0)&(y<480))
  z=truth[eye+'_depth_m'][y,x];T=truth[eye+'_camera_usd_world'];R=T[:3,:3]@np.diag([1,-1,-1]);rays=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))]
  with np.errstate(invalid='ignore'):w=T[:3,3]+(rays*z[:,None])@R.T;err.append(np.linalg.norm(w-points,axis=1))
 e=np.asarray(err).T;finite=np.isfinite(e).all(1);ok=finite&np.all(e<=.1,axis=1)
 return dict(n=len(points),correct=int(ok.sum()),insufficient=int((~finite).sum()),finite_wrong=int((finite&~ok).sum()),precision=float(ok.mean()) if len(ok) else None)
for r in json.loads((D/'records.json').read_text()):
 i=r['frame'];truth=np.load(O/'offline_truth'/f'frame_{i:03d}.npz');groups=[];world=[];ids=[]
 for g in r['groups']:
  p=np.load(D/g['file']);s=p['confirmed_four_view'];assert int(s.sum())==g['confirmed'];assert float(p['source_geometry_time_s'])<r['time_s'];sc=score(p['past_world'][s],[p['left_uv'][s],p['right_uv'][s]],truth)
  groups.append(dict(source_frame=g['source_frame'],source_time_s=g['source_time_s'],age_s=r['time_s']-g['source_time_s'],**sc));allgroups.append(groups[-1])
  world.extend(p['past_world'][s]);ids.extend((g['source_frame'],int(q),int(z)) for q,z in zip(p['past_query_id'][s],p['past_partner_id'][s]))
 w=np.asarray(world).reshape(-1,3);bins=np.floor(w[:,:2]/.1).astype(int);novel=np.array([tuple(b) not in initial_bins for b in bins],dtype=bool);nbins=set(map(tuple,bins[novel]));speed=float(np.linalg.norm(states[i]['velocity'][:2]));dist=tree.query(w[:,:2])[0] if len(w) else np.array([])
 rows.append(dict(frame=i,time_s=r['time_s'],groups=groups,confirmed_pixels=len(w),unique_source_ids=len(set(ids)),unique_XY_bins=len(set(map(tuple,bins))),novel_bin_pixels=int(novel.sum()),novel_XY_bins=len(nbins),novel_bins=[[int(x),int(y)] for x,y in sorted(nbins)],measured_XY_speed_m_s=speed,new_region_coverage_gate=bool(r['time_s']>=18 and speed>=.05 and novel.sum()>=20 and len(nbins)>=2),descriptive_not_new_gate_distance_ge10cm_pixels=int((dist>=.1).sum()),descriptive_max_nearest_seed_XY_m=float(dist.max()) if len(dist) else None))
valid=[g for g in allgroups if g['n']];precision=bool(valid) and all(g['precision']>=.95 for g in valid);coverage=any(r['new_region_coverage_gate'] for r in rows);result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_READONLY_NEW_REGION_DIAGNOSTIC_ONLY' if precision and coverage else 'REJECT_FOUR_VIEW_NEW_REGION_RULE',every_nonempty_group_precision_ge95=precision,moving_new_region_coverage_gate=coverage,nonempty_groups=len(valid),aggregate_points=sum(g['n'] for g in valid),aggregate_correct=sum(g['correct'] for g in valid),aggregate_insufficient=sum(g['insufficient'] for g in valid),aggregate_finite_wrong=sum(g['finite_wrong'] for g in valid),rows=rows,labels_read_only_after_prediction_seal=True,assessor_sha256=sha(__file__),prediction_seal_sha256=sha(D/'prediction_seal.json'),knownpose_privileged=True,development_not_fresh=True,new_control_admitted=False,whole_research_complete=False)
f=D/'assessment_v1.json';assert not f.exists();f.write_text(json.dumps(result,indent=2)+'\n');print({k:v for k,v in result.items() if k!='rows'});print([(r['frame'],r['confirmed_pixels'],r['novel_bin_pixels'],r['novel_XY_bins'],r['measured_XY_speed_m_s'],r['new_region_coverage_gate']) for r in rows if r['confirmed_pixels']]);print('failed_groups',[(r['frame'],g['source_frame'],g['n'],g['precision']) for r in rows for g in r['groups'] if g['n'] and g['precision']<.95])
