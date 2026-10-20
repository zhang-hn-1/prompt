from pathlib import Path
import json,numpy as np
from measurement import geometry,bank
h=Path(__file__).resolve().parent;old=h.parent/'oracle_bank_mapping_20260929_v2'
def measure(p,fresh=False):
 cfg=json.loads((p/'config.json').read_text());main=cfg['main'];z=np.load(p/'physics.npz');t=z['simulation_time_s'];v,k=geometry(z['position_world_xy']);k*=np.sign(main);b=np.degrees(bank(z['root_quaternion_world_wxyz']));fb=json.loads(next(p.glob('bank_feedback_*.json')).read_text());duration=12 if fresh else 15
 targets=[.3,-.6,.5,-.3] if fresh else [.4,-.4,.7,-.7];windows=[]
 for j,target in enumerate(targets):
  start=5+j*duration;end=start+duration;trans=(t>=start+.02)&(t<=start+8)&np.isfinite(k);tail=(t>=end-5)&(t<=end-.5);valid=tail&np.isfinite(k);transi=np.flatnonzero(trans);error=k[trans]-target;actual=float(np.median(k[valid]));threshold=max(.2,.3*abs(target));band=(t>=start+.02)&(t<=end-.5)&np.isfinite(k);bad=np.flatnonzero(band&(abs(k-target)>threshold));bankerr=np.array([fb[i]['error_rad'] for i in transi if i<len(fb)]);signed=z['linear_velocity_world_m_s'][:,0]*np.cos(z['yaw_world_rad'])+z['linear_velocity_world_m_s'][:,1]*np.sin(z['yaw_world_rad']);direction=float(np.mean(signed[valid]*main>0))
  window_mask=(t>=start+.02)&(t<=start+8);roll_rate=np.gradient(b,t);windows.append(dict(start_s=start,target=target,transient_valid_fraction=float(trans.sum()/window_mask.sum()),transient_bank_rate_rms_deg_s=float(np.sqrt(np.mean(roll_rate[trans]**2))),tail_bank_span_deg=float(np.ptp(np.percentile(b[valid],[5,95]))),transient_mse=float(np.mean(error**2)),transient_bank_mse_deg2=float(np.mean(np.degrees(bankerr)**2)),settling_s=float(t[bad[-1]]-start+.02) if len(bad) else 0,tail_curvature=actual,tail_error=abs(actual-target),tail_gate=bool(abs(actual-target)<=threshold and actual*target>0 and valid.sum()/tail.sum()>=.9 and direction>=.9),direction_match=direction))
 return dict(case=p.name,main=main,gains=cfg['gains'],windows=windows,finite=bool(all(np.isfinite(z[k]).all() for k in z.files)),max_bank_deg=float(max(abs(b))),curvature_rms=float(np.sqrt(np.mean([w['transient_mse'] for w in windows]))),bank_rms_deg=float(np.sqrt(np.mean([w['transient_bank_mse_deg2'] for w in windows]))),mean_settling_s=float(np.mean([w['settling_s'] for w in windows])),all_tail_pass=all(w['tail_gate'] for w in windows),supervisor_fraction=float(np.mean([r['supervisor_active'] for r in fb])),pid_clip_fraction=float(np.mean([r['pid_clipped'] for r in fb])))
if __name__=='__main__':
 baseline=[measure(old/f'validation_{m}') for m in [1.,-1.]];base_k=float(np.sqrt(np.mean([r['curvature_rms']**2 for r in baseline])));base_b=float(np.sqrt(np.mean([r['bank_rms_deg']**2 for r in baseline])));groups=[]
 for tag in ['D070','D105','P040D070','P120D070']:
  if not all((h/f'{tag}_{m}/physics.npz').exists() for m in [1.,-1.]):continue
  rr=[measure(h/f'{tag}_{m}') for m in [1.,-1.]];k=float(np.sqrt(np.mean([r['curvature_rms']**2 for r in rr])));b=float(np.sqrt(np.mean([r['bank_rms_deg']**2 for r in rr])));checks=dict(curvature_improves_15pct=k<=.85*base_k,bank_not_worse_10pct=b<=1.1*base_b,tail_pass=all(r['all_tail_pass'] for r in rr),finite=all(r['finite'] for r in rr),bank_below35=all(r['max_bank_deg']<35 for r in rr))
  groups.append(dict(tag=tag,gains=rr[0]['gains'],rows=rr,curvature_rms=k,bank_rms_deg=b,improvement_fraction=1-k/base_k,checks=checks,eligible=all(checks.values())))
 result=dict(baseline=baseline,baseline_curvature_rms=float(base_k),baseline_bank_rms_deg=float(base_b),candidates=groups)
 if (h/'supervisor060_1.0/physics.npz').exists():result['supervisor_sensitivity']=measure(h/'supervisor060_1.0')
 (h/'tuning_assessment.json').write_text(json.dumps(result,indent=2));print('BASE',base_k,base_b)
 for r in groups:print(r['tag'],r['curvature_rms'],r['bank_rms_deg'],r['eligible'],r['checks'])
 if len(groups)==4:
  eligible=[r for r in groups if r['eligible']];best=min(eligible or groups,key=lambda r:r['curvature_rms']);kp,ki,kd=map(float,best['gains'].split(','));(h/'selected_candidate.json').write_text(json.dumps(dict(tag=best['tag'],gains=dict(kp=kp,ki=ki,kd=kd),tuning_eligible=bool(eligible),selection='minimum transient curvature RMS among eligible; if none diagnostic-only best',improvement_fraction=best['improvement_fraction']),indent=2))
