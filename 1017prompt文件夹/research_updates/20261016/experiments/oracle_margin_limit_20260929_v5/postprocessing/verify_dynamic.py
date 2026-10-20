from pathlib import Path
import sys,json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
h=Path('/home/an/Documents/ChatGPT/球型机器人导航/experiments/oracle_margin_limit_20260929_v5');sys.path.insert(0,str(h));from measurement import geometry,bank
from roll_pid import BankPID
old=h.parent/'oracle_pid_validation_20260929_v4';out=[];fig,axs=plt.subplots(2,2,figsize=(12,8))
for col,m in enumerate([.5,-1.5]):
 ref=np.load(old/f'fresh_selected_{m}/physics.npz');zs=[]
 for tag,p,label in [('base',old/f'fresh_selected_{m}','limit 0.30'),('relax',h/f'limit060_{m}','limit 0.60'),('off',h/f'limitoff_{m}','supervisor off')]:
  z=np.load(p/'physics.npz');zs.append(z);t=z['simulation_time_s'];_,k=geometry(z['position_world_xy']);axs[0,col].plot(t,k*np.sign(m),label=label,linestyle='--' if tag=='off' else '-');axs[1,col].plot(t,np.degrees(bank(z['root_quaternion_world_wxyz'])),label=label,linestyle='--' if tag=='off' else '-')
  initial={key:bool(np.array_equal(z[key][0],ref[key][0])) for key in ['position_world_xy','root_quaternion_world_wxyz','linear_velocity_world_m_s','angular_velocity_world_rad_s','joint_position_rad','joint_velocity_rad_s','applied_action']};assert all(initial.values())
  fb=json.loads(next(p.glob('bank_feedback_*.json')).read_text());pid=BankPID(kp=.4,ki=.1,kd=.7);blocked=False;errs=[]
  for r in fb:
   pid.step([r['main_request_rad_s'],r['aux_feedforward_rad']],r['bank_rad'],downstream_blocked=blocked);errs.append(abs(r['pid_protocol_aux_rad']-pid.last['pid_protocol_aux_rad']));blocked=r['supervisor_active']
  assert len(fb)==2650 and max(errs)<1e-12
  out.append(dict(case=p.name,initial_identical=initial,pid_ticks=len(fb),pid_max_error=max(errs),supervisor_fraction=float(np.mean([r['supervisor_active'] for r in fb]))))
 identical={key:bool(np.array_equal(zs[1][key],zs[2][key])) for key in zs[1].files};out.append(dict(main=m,limit060_off_identical=identical));assert all(v for k,v in identical.items() if k not in ['raw_simulation_time_s','raw_physics_step_index'])
 axs[0,col].set_title(f'Main {m} rad/s');axs[0,col].set_ylabel('body curvature [1/m]');axs[1,col].set_ylabel('physical bank [deg]')
 for ax in axs[:,col]:ax.set_xlabel('simulation time [s]');ax.grid();ax.legend()
fig.tight_layout();fig.savefig(h/'limits_comparison.png',dpi=160);(h/'dynamic_execution_audit.json').write_text(json.dumps(out,indent=2));print('Exact same .60/off physics, matching initial states, PID output replay verified')
