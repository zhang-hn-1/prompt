from pathlib import Path
import json,numpy as np
H=Path(__file__).resolve().parent;D=H/'E1_continuous_tube_navigation_run_v1';p=json.loads((D/'policy_records.json').read_text());s=json.loads((D/'states.json').read_text());xy=np.asarray([x['position'][:2] for x in s]);t=np.asarray([x['time_s'] for x in s]);req=np.asarray([x['request'] for x in p]);speed=np.linalg.norm(np.asarray([x['velocity'][:2] for x in s]),axis=1)
def key(r):
 st=r['tracker_state_after_act'];return (r['local_reference_version'],st['active_direction'],st['pending_direction'],st['braking_state'],int(np.sign(r['request'][0])))
def project(q,ref):
 v=np.diff(ref,axis=0);a=ref[:-1];l=np.linalg.norm(v,axis=1);z=np.sum(v*v,axis=1);f=np.divide(np.sum((q-a)*v,axis=1),z,out=np.zeros(len(z)),where=z>0).clip(0,1);d=np.linalg.norm(q-a-f[:,None]*v,axis=1);j=np.argmin(d);return float(sum(l[:j])+f[j]*l[j]),float(d[j])
start=[0]+[i for i in range(1,len(p)) if key(p[i])!=key(p[i-1])]+[len(p)];out=[]
for b,e in zip(start[:-1],start[1:]):
 ref=np.asarray(p[b]['installed_reference_xy_after_act'] or [],float).reshape(-1,2);pairs=np.asarray([project(q,ref) for q in xy[b:e+1]]) if len(ref)>1 else None
 out.append(dict(begin=b,end=e,seconds=float(t[e]-t[b]),state=key(p[b]),world_displacement_m=float(np.linalg.norm(xy[e]-xy[b])),world_path_m=float(np.linalg.norm(np.diff(xy[b:e+1],axis=0),axis=1).sum()),speed_start=float(speed[b]),speed_end=float(speed[e]),maximum_speed=float(max(speed[b:e+1])),net_frozen_reference_progress_m=None if pairs is None else float(pairs[-1,0]-pairs[0,0]),cross_track_peak_m=None if pairs is None else float(max(pairs[:,1])),aux_boundary_request_fraction=float(np.mean(abs(req[b:e,1])>=np.deg2rad(30)-1e-7)),path_index_start=p[b]['tracker_state_after_act']['path_index'],path_index_end=p[e-1]['tracker_state_after_act']['path_index']))
result=dict(scope='POSTCLOSE descriptive development physical phases, labels from actual current state. Boundaries reference/active/pending/braking/main sign; no policy act recall. Nearest full frozen reference projection independent cursor, progress NOT sum across refs.',phases=out,formal35_NOTRUN=True)
(D/'physical_phase_audit_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps([r for r in out if r['begin']>=381]))
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,ax=plt.subplots(1,2,figsize=(13,5));ax[0].plot(xy[:,0],xy[:,1],label='actual XY');
for i in [116,296,386,511]:
 r=np.asarray(p[i]['installed_reference_xy_after_act']);ax[0].plot(r[:,0],r[:,1],'--',alpha=.5,label=f'ref at {i/10:.1f}s')
ax[0].scatter(xy[436,0],xy[436,1],label='first request fork43.6s');ax[0].set_aspect('equal');ax[0].legend(fontsize=7);ax[0].set_xlabel('world X m');ax[0].set_ylabel('world Y m');ax[1].plot(t[:-1],req[:,0],label='main request');ax[1].plot(t,speed,label='actual XY speed');ax[1].axvline(t[436],color='k',ls=':');ax[1].legend();ax[1].set_xlabel('simulation seconds');fig.tight_layout();fig.savefig(D/'trajectory_and_phase_v1.png',dpi=150);plt.close(fig)
