from pathlib import Path
import json,hashlib,numpy as np
from scipy.spatial.transform import Rotation
from measurement import bank,geometry
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
h=Path(__file__).resolve().parent;src=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/runs/benchmark_round4_drive_calibration_20260920_v1/traces');rows=[];series={};hashes={}
for p in sorted(src.glob('*.json')):
 r=json.loads(p.read_text());s=r['samples'];t=np.array([v['time_s'] for v in s]);xy=np.array([v['position_xy_m'] for v in s]);q=np.array([v['root_quaternion_world_wxyz'] for v in s]);beta=bank(q);speed,k=geometry(xy);steer=np.array([v['phase'] in ['steer','drive'] for v in s]);end=t[steer].max();win=(t>=end-2.5)&(t<=end-.5);valid=win&np.isfinite(k);span=lambda a:float(np.quantile(a,.95)-np.quantile(a,.05));bs=span(np.degrees(beta[win]));vs=span(speed[win]);vm=float(np.median(speed[win]));stable=bool(bs<=3 and vs<=.2*vm and valid.sum()>=.9*win.sum());sens={}
 for w in [21,31,41]:
  _,kk=geometry(xy,window=w);sens[str(w)]=float(np.nanmedian(kk[win])) if np.isfinite(kk[win]).any() else None
 case=r['case'];row={'case_id':case['case_id'],'historical_role':case['dataset_role'],'main_command':case['main_command_rad_s'],'auxiliary_command_deg':float(np.degrees(case['auxiliary_command_rad'])),'window_s':[float(t[win].min()),float(t[win].max())],'speed_m_s':vm,'bank_deg':float(np.median(np.degrees(beta[win]))),'bank_span_deg':bs,'speed_span_m_s':vs,'valid_fraction':float(valid.sum()/win.sum()),'quasi_steady':stable,'curvature_m_inv':float(np.median(k[valid])) if valid.any() else None,'curvature_iqr':float(np.quantile(k[valid],.75)-np.quantile(k[valid],.25)) if valid.any() else None,'actual_auxiliary_deg':float(np.median([np.degrees(s[i]['auxiliary_joint_rad']) for i in np.flatnonzero(win)])),'smoothing_sensitivity':sens};rows.append(row);series[case['case_id']]={'t':t,'beta':beta,'speed':speed,'k':k,'win':win,'aux':np.array([v['auxiliary_joint_rad'] for v in s])};hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
# Exact current-asset body identity and shared-axis check.
p=h.parent/'oracle_slip_audit_20260928_v1/LR_candidate';z=np.load(p/'links.npz');phys=np.load(p/'physics.npz');poses=z['poses'][:,0];qb=poses[:,0,3:7][:,[3,0,1,2]];qs=poses[:,1,3:7][:,[3,0,1,2]];bb=bank(qb);sb=bank(qs);roll=phys['roll_world_rad'];diff=np.degrees(bb-roll);identity={'root_base_quaternion_max_diff':float(abs(qb-phys['root_quaternion_world_wxyz']).max()),'root_shell_center_max_distance_m':float(np.linalg.norm(poses[:,0,:3]-poses[:,1,:3],axis=1).max()),'base_shell_bank_max_diff_deg':float(np.degrees(abs(bb-sb)).max()),'base_shell_bank_p95_diff_deg':float(np.quantile(np.degrees(abs(bb-sb)),.95)),'bank_roll_abs_diff_max_deg':float(abs(diff).max()),'bank_roll_abs_diff_p95_deg':float(np.quantile(abs(diff),.95))}
for f in [p/'links.npz',p/'physics.npz']:hashes[str(f)]=hashlib.sha256(f.read_bytes()).hexdigest()
(h/'source_hashes.json').write_text(json.dumps(hashes,indent=2));(h/'assessment.json').write_text(json.dumps({'identity':identity,'n_traces':len(rows),'quasi_steady_count':sum(r['quasi_steady'] for r in rows),'rows':rows},indent=2));print(json.dumps({'identity':identity,'quasi_steady_count':sum(r['quasi_steady'] for r in rows),'rows':[{k:r[k] for k in ['case_id','speed_m_s','bank_deg','curvature_m_inv','quasi_steady','bank_span_deg']} for r in rows]},indent=2))
fig,ax=plt.subplots(1,2,figsize=(12,5))
for r in rows:
 if r['curvature_m_inv'] is None:continue
 marker='o' if r['quasi_steady'] else 'x'
 dots=ax[0].scatter(r['bank_deg'],r['curvature_m_inv'],c=[r['speed_m_s']],vmin=0,vmax=.65,cmap='viridis',marker=marker)
colors={-30:'navy',-15:'royalblue',0:'gray',15:'orange',30:'firebrick'}
for aux,color in colors.items():
 rr=[r for r in rows if r['historical_role']=='fit_core' and round(r['auxiliary_command_deg'])==aux]
 ax[1].scatter([r['speed_m_s'] for r in rr],[abs(r['curvature_m_inv']) for r in rr],color=color,label=f'aux request {aux} deg')
fig.colorbar(dots,ax=ax[0],label='Center speed (m/s)')
ax[0].set(xlabel='Common axle bank (deg)',ylabel='Signed XY curvature (1/m)',title='Historical windows: o=bank/speed quasi-steady, x=other');ax[1].set(xlabel='Measured center speed (m/s)',ylabel='Absolute XY curvature (1/m)',title='Core windows incl. oscillation; NOT steady map');ax[1].legend(fontsize=7)
for a in ax:a.grid(alpha=.3)
fig.tight_layout();fig.savefig(h/'relationship.png',dpi=160)
fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
for name in ['core_m0p50_ap30','core_m1p50_ap30']:
 s=series[name];axes[0].plot(s['t'],np.degrees(s['beta']),label=name+' bank');axes[0].plot(s['t'],np.degrees(s['aux']),ls='--',alpha=.5,label=name+' actual aux');axes[1].plot(s['t'],s['speed'],label=name);axes[2].plot(s['t'],s['k'],label=name)
for a in axes:a.grid(alpha=.3);a.legend(fontsize=7)
axes[0].set_ylabel('deg');axes[1].set_ylabel('speed m/s');axes[2].set_ylabel('curvature 1/m');axes[2].set_xlabel('Time (s)');axes[2].set_ylim(-10,10);fig.tight_layout();fig.savefig(h/'transitions.png',dpi=160)
