"""Post-run evaluation; thresholds are from the pre-run PROTOCOL."""
import json,math,hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from benchmark.drive_model import DriveModel
h=Path(__file__).resolve().parent;root=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
m=DriveModel.from_json(root/'configs/benchmark_v1/drive_dynamics_v1.json')
out={};fig,axes=plt.subplots(2,2,figsize=(11,9))
for row,kind in enumerate(['LR','RL']):
 p=h/(kind+'_measure');q=h/(kind+'_repeat');z=np.load(p/'physics.npz');zr=np.load(q/'physics.npz');s=400
 xy=z['position_world_xy'][s:];yaw=np.unwrap(z['yaw_world_rad'][s:]);t=z['simulation_time_s'][s:]-z['simulation_time_s'][s]
 theta=float(yaw[0]);R=np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]]);local=(xy-xy[0])@R
 cfg=json.loads((p/'config.json').read_text());aux=cfg['angles'][1];v=m.speed(1.5,aux);k=m.curvature(1.5,aux);ds=t*v
 if abs(k)<1e-10:ideal=np.c_[ds,ds*0]
 else:ideal=np.c_[np.sin(k*ds)/k,(1-np.cos(k*ds))/k]
 instant_error=np.linalg.norm(ideal-local,axis=1)
 eligible=json.loads((q/'prediction_status.json').read_text())['eligible']
 if eligible:
  pred=np.load(q/'prediction_before_transition.npz')['position_world_xy'];diff=np.linalg.norm(pred-zr['position_world_xy'][s:],axis=1)
  angle=np.abs(np.arctan2(np.sin(z['yaw_world_rad'][s:]-zr['yaw_world_rad'][s:]),np.cos(z['yaw_world_rad'][s:]-zr['yaw_world_rad'][s:])))
  joint=np.abs(z['joint_position_rad'][s:,1]-zr['joint_position_rad'][s:,1]);main=np.abs(z['joint_velocity_rad_s'][s:,0]-zr['joint_velocity_rad_s'][s:,0])
  accepted=bool(diff.max()<=.02 and angle.max()<=math.radians(2) and joint.max()<=math.radians(2) and main.max()<=.05)
  repeat=dict(max_xy_error_m=float(diff.max()),max_yaw_error_deg=float(np.degrees(angle.max())),max_aux_error_deg=float(np.degrees(joint.max())),max_main_speed_error_rad_s=float(main.max()),accepted_exact_state_repeatability=accepted)
 else:repeat=dict(accepted_exact_state_repeatability=False)
 np.savez(h/(kind+'_measured_primitive.npz'),time_s=t,local_xy=local,body_yaw_delta_rad=yaw-yaw[0],joint_position_rad=z['joint_position_rad'][s:],joint_velocity_rad_s=z['joint_velocity_rad_s'][s:],raw_target=z['raw_command_target'][s:],smoothed_target=z['smoothed_target'][s:],applied_target=z['applied_action'][s:],linear_velocity_world_m_s=z['linear_velocity_world_m_s'][s:])
 horizons={str(H):{'instant_xy_error_m':float(instant_error[int(round(H/.02))]),'measured_displacement_m':float(np.linalg.norm(local[int(round(H/.02))])),'measured_aux_deg':float(np.degrees(z['joint_position_rad'][s+int(round(H/.02)),1]))} for H in [.5,1.,1.5,2.,4.]}
 out[kind]={'eligible':eligible,'repeat':repeat,'instant_V1_speed_m_s':v,'instant_V1_curvature_1_m':k,'horizons':horizons,'start_aux_deg':float(np.degrees(z['joint_position_rad'][s,1])),'start_main_rad_s':float(z['joint_velocity_rad_s'][s,0]),'end_local_xy_m':local[-1].tolist(),'end_body_yaw_delta_deg':float(np.degrees(yaw[-1]-yaw[0])),'scope':'exact state reproducibility only; not generalization or navigation validation'}
 axes[row,0].plot(local[:,0],local[:,1],label='measured transition');axes[row,0].plot(ideal[:,0],ideal[:,1],'--',label='instant V1 arc');axes[row,0].axis('equal');axes[row,0].set_title(kind+' local XY');axes[row,0].set_xlabel('m');axes[row,0].set_ylabel('m')
 axes[row,1].plot(t,np.degrees(z['joint_position_rad'][s:,1]),label='actual aux');axes[row,1].plot(t,np.degrees(z['applied_action'][s:,1]),label='applied target');axes[row,1].axhline(np.degrees(aux),ls='--',label='request');axes[row,1].set_title(kind+' steering');axes[row,1].set_xlabel('s');axes[row,1].set_ylabel('deg')
 for ax in axes[row]:ax.legend();ax.grid(alpha=.3)
for file in ['pre_run_identity.json','frozen_templates.json']:
 manifest=json.loads((h/file).read_text());assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in manifest.items())
(h/'assessment.json').write_text(json.dumps(out,indent=2)+'\n');fig.tight_layout();fig.savefig(h/'comparison.png',dpi=150)
print(json.dumps(out,indent=2))
