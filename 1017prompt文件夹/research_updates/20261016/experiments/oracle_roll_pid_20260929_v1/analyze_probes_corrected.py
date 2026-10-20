"""Corrected audit: recorded scored time; preserves original preselection artifacts."""
from pathlib import Path
import json,numpy as np
from roll_pid import bank_from_quaternion
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
h=Path(__file__).resolve().parent;rows=[]
fig,axes=plt.subplots(2,1,figsize=(11,7),sharex=True)
for p in sorted(h.glob('probe_*/physics.npz')):
 z=np.load(p);t=z['simulation_time_s'];b=np.degrees([bank_from_quaternion(q) for q in z['root_quaternion_world_wxyz']]);speed=np.linalg.norm(z['linear_velocity_world_m_s'][:,:2],axis=1)
 c=json.loads((p.parent/'config.json').read_text());windows=[]
 for start,end,target in [(12,15,15),(22,25,-15),(32,35,0)]:
  mask=(t>start+1e-8)&(t<=end+1e-8);x=b[mask];windows.append(dict(start_s=start,end_s=end,n=int(mask.sum()),rms_error_deg=float(np.sqrt(np.mean((x-target)**2))),peak_error_deg=float(np.max(abs(x-target))),span_deg=float(np.ptp(x)),mean_bank_deg=float(x.mean()),actual_speed_mean_m_s=float(speed[mask].mean()),rate_rms_deg_s=float(np.sqrt(np.mean((np.diff(x)/np.diff(t[mask]))**2)))))
 fb=list(p.parent.glob('bank_feedback_*.json'));recs=json.loads(fb[0].read_text()) if fb else []
 row=dict(case=p.parent.name,main=c['main'],mode=c['mode'],all_physics_fields_finite=bool(all(np.isfinite(z[k]).all() for k in z.files)),samples=len(t),start_time_s=float(t[0]),end_time_s=float(t[-1]),time_step_ok=bool(np.allclose(np.diff(t),.02)),max_abs_bank_deg=float(max(abs(b))),windows=windows,tail_rms_deg=float(np.sqrt(np.mean([w['rms_error_deg']**2 for w in windows]))),supervisor_fraction=float(np.mean(abs(z['applied_action'][1:,1]-z['smoothed_target'][1:,1])>1e-6)),original_protocol_clip_fraction=float(np.mean(abs(z['raw_command_target'][1:,1]-z['clipped_target'][1:,1])>1e-6)),pid_clip_fraction=float(np.mean([r['pid_clipped'] for r in recs])) if recs else None)
 rows.append(row);ax=axes[0 if c['main']==.5 else 1];ax.plot(t,b,label=c['mode'].upper());ax.set_ylabel(f"main={c['main']} rad/s\nbank [deg]");ax.legend();ax.grid(True)
for ax in axes:ax.plot([0,5,5,15,15,25,25,35],[0,0,15,15,-15,-15,0,0],'k--',alpha=.5)
axes[-1].set_xlabel('recorded simulation time [s]');fig.tight_layout();fig.savefig(h/'bank_probe_corrected.png',dpi=140)
gates=[]
for speed in [.5,1.5]:
 b=next(r for r in rows if r['main']==speed and r['mode']=='b0');p=next(r for r in rows if r['main']==speed and r['mode']=='pid')
 checks=dict(finite=p['all_physics_fields_finite'],valid_time=p['time_step_ok'],bank_below_35_deg=p['max_abs_bank_deg']<35,all_tail_spans_below_10_deg=all(w['span_deg']<10 for w in p['windows']),tail_rms_not_worse=p['tail_rms_deg']<=b['tail_rms_deg'],upright_rms_not_2deg_worse=p['windows'][-1]['rms_error_deg']<=b['windows'][-1]['rms_error_deg']+2)
 gates.append(dict(main=speed,checks=checks,pass_all=all(checks.values()),rms_reduction_fraction=1-p['tail_rms_deg']/b['tail_rms_deg']))
result=dict(correction='Initial diagnostic sample is t=0, not .02. Original analysis/selection preserved. No gains changed. Four runs form two pairs.',rows=rows,gates=gates)
(h/'probe_assessment_corrected.json').write_text(json.dumps(result,indent=2));print(json.dumps(gates,indent=2));assert all(g['pass_all'] for g in gates)
