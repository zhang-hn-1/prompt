from pathlib import Path
import json,numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
H=Path(__file__).resolve().parents[1];D=H/'connected_surface_memory_v1';R=H/'connected_surface_memory_reviewer1_v1';O=H/'qualified_hazard_navigation_run_v1';records=json.loads((D/'records.json').read_text());seed=np.load(O/'sensor_packets/source_180.npz');sw=seed['world'];sw=sw[(sw[:,2]>.05)&(sw[:,2]<=1.5)];history={a:[(float(seed['time_s']),sw)] for a in ('F0','F1')};sb={tuple(x) for x in np.floor(sw[:,:2]/.1).astype(int)};checks=[];scores=[];bins={a:set() for a in history}
for frame in (235,245,255):
 rows=[r for r in records if r['frame']==frame];packets=[np.load(D/r['file']) for r in rows];world=np.vstack([f['world'] for f in packets]);v=np.concatenate([f['bilinear_patch_contract_valid'] for f in packets]);q=world[v];t=float(packets[0]['current_time_s'])
 for arm in history:
  old=[(age,p) for age,p in history[arm] if 0<=t-age<=10];anchors=np.vstack([p for age,p in old]);root=cKDTree(anchors).query(q)[0]<=.1
  if arm=='F1':
   pairs=cKDTree(q).query_pairs(.1,output_type='ndarray');adj=coo_matrix((np.ones(len(pairs)),(pairs[:,0],pairs[:,1])),shape=(len(q),len(q)));_,labels=connected_components(adj,directed=False);keep=np.isin(labels,np.unique(labels[root]))
  else:keep=root
  accept=np.zeros(len(world),bool);accept[v]=keep;saved=np.concatenate([f[arm+'_direct_accepted' if arm=='F0' else arm+'_connected_accepted'] for f in packets]);assert np.array_equal(accept,saved);history[arm]=old+[(t,world[accept])];checks.append(dict(frame=frame,arm=arm,accepted=int(accept.sum()),anchors=len(anchors),independent_component_labels_equal=True))
 for r,f in zip(rows,packets):
  errors=[]
  for source_frame,uv in [(frame,f['current_uv']),*zip(f['source_frames'],f['past_uv'])]:
   truth=np.load(O/'offline_truth'/f'frame_{source_frame:03d}.npz');p=np.rint(uv).astype(int);z=truth[r['eye']+'_depth_m'][p[:,1],p[:,0]];rays=np.stack([(p[:,0]-319.5)/584.8888888888889,-(p[:,1]-239.5)/584.8888888888889,-np.ones(len(p))],axis=1);T=truth[r['eye']+'_camera_usd_world']
   with np.errstate(invalid='ignore'):actual=(rays*z[:,None])@T[:3,:3].T+T[:3,3];errors.append(np.linalg.norm(actual-f['world'],axis=1))
  e=np.stack(errors,axis=1);finite=np.isfinite(e).all(1);correct=finite&(e<=.1).all(1);b=np.floor(f['world'][:,:2]/.1).astype(int);nov=np.array([tuple(x) not in sb for x in b]);
  for arm in history:
   select=f[arm+'_direct_accepted' if arm=='F0' else arm+'_connected_accepted'];n=int(select.sum());good=correct&select;new=good&nov;newbins={tuple(x) for x in b[new]};post=r['time_s']>23.5
   if post:bins[arm].update(newbins)
   a=next(a for a in json.loads((D/'assessment_v1.json').read_text())['rows'] if a['frame']==frame and a['eye']==r['eye'] and a['arm']==arm);observed=dict(n=n,correct=int(good.sum()),insufficient=int((select&~finite).sum()),finite_wrong=int((select&finite&~correct).sum()),precision=float(good.sum()/n) if n else None,correct_novel_points=int(new.sum()),correct_novel_bins=len(newbins),strict_coverage=bool(post and new.sum()>=20 and len(newbins)>=2));assert all(a[k]==v for k,v in observed.items());scores.append(dict(frame=frame,eye=r['eye'],arm=arm,**observed))
result=dict(graph_checks=checks,independent_scores=scores,post235_bins={a:len(b) for a,b in bins.items()},full_scoring_equals_author=True);(R/'independent_oracle.json').write_text(json.dumps(result,indent=2)+'\n');print('Independent sparse connected-component closure and three-endpoint scoring equal all six packets / twelve rows')
