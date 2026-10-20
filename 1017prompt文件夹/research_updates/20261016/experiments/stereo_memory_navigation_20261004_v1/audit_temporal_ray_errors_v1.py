"""One readonly error decomposition, no model/threshold change."""
from pathlib import Path
import numpy as np,json,datetime,hashlib
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;O=H/'qualified_hazard_navigation_run_v1';D=H/'temporal_ray_sweep_diagnostic_v1';a=json.loads((D/'assessment_v1.json').read_text());fig,ax=plt.subplots(3,2,figsize=(12,14));rows=[]
for n,r in enumerate(a['rows']):
 p=np.load(D/r['file']);w=p['world'];uvs=[p['current_uv'],*p['past_uv']];frames=[r['frame'],*p['source_frames']];errors=[]
 for i,uv in zip(frames,uvs):
  truth=np.load(O/'offline_truth'/f'frame_{i:03d}.npz');x,y=np.rint(uv).astype(int).T;T=truth[r['eye']+'_camera_usd_world'];z=truth[r['eye']+'_depth_m'][y,x];ray=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))]
  with np.errstate(invalid='ignore'):world=T[:3,3]+(ray*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;errors.append(np.linalg.norm(world-w,axis=1))
 e=np.array(errors).T;finite=np.isfinite(e).all(1);ok=finite&np.all(e<=.1,axis=1);wrong=finite&~ok;insuff=~finite
 currentbad=np.isfinite(e[:,0])&(e[:,0]>.1);pastbad=np.isfinite(e[:,1:]).all(1)&np.any(e[:,1:]>.1,axis=1);rows.append(dict(frame=r['frame'],eye=r['eye'],current_finite_wrong=int(currentbad.sum()),past_finite_wrong_any=int(pastbad.sum()),current_and_past_finite_wrong=int((currentbad&pastbad).sum()),finite_joint_wrong_current_ok_past_bad=int((wrong&(e[:,0]<=.1)).sum()),insufficient_current=int((~np.isfinite(e[:,0])).sum()),insufficient_past_any=int((~np.isfinite(e[:,1:]).all(1)).sum()),correct=int(ok.sum()),finite_wrong=int(wrong.sum())))
 image=np.load(O/'online_inputs'/f'frame_{r["frame"]:03d}.npz')[r['eye']+'_rgb'];plot=ax[n//2,n%2];plot.imshow(image)
 for s,color,name in [(ok,'lime','all3 correct'),(wrong,'red','finite wrong'),(insuff,'magenta','label insufficient')]:
  if s.any():plot.scatter(p['current_uv'][s,0],p['current_uv'][s,1],s=3,c=color,label=name,alpha=.6)
 plot.set_title(f"{r['time_s']:.1f}s {r['eye']}: {r['correct']}/{r['n']} ({r['precision']:.1%})");plot.axis('off');plot.legend(loc='upper right',fontsize=7)
fig.suptitle('REJECT: temporal ray sweep, offline three-endpoint labels (known poses)',fontsize=13);fig.tight_layout();fig.savefig(D/'temporal_ray_sweep_error_overlays_v1.png',dpi=120);plt.close(fig)
res=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),labels_offline_only=True,no_rescoring_or_candidate_change=True,aggregate_correct_not_new_gate=True,full_recursion_or_control_not_validated=True)
f=D/'error_decomposition_v1.json';assert not f.exists();f.write_text(json.dumps(res,indent=2)+'\n');print(rows)
