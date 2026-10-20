import json,hashlib,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
h=Path(__file__).resolve().parent;old=h.parent/'oracle_measured_turn_primitives_20260928_v1';out={};fig,axes=plt.subplots(2,2,figsize=(11,9))
def wrap(x):return np.arctan2(np.sin(x),np.cos(x))
def yaw(q):return math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]**2+q[3]**2))
def levelvec(v,q):
 a=yaw(q);c,s=math.cos(a),math.sin(a);x,y,z=v;return np.array([c*x+s*y,-s*x+c*y,z])
def rp(q):
 w,x,y,z=q;return np.array([math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),math.asin(np.clip(2*(w*y-z*x),-1,1))])
for row,kind in enumerate(['LR','RL']):
 p=h/f'{kind}_validation';opp=kind;source=old/f'{opp}_measure'
 z=np.load(p/'physics.npz');prefix=np.load(h.parent/'oracle_chained_turn_primitives_20260928_v1'/f'{kind}_chain'/'physics.npz');template=np.load(source/'physics.npz');pred=np.load(p/'prediction_before_chain.npz')
 a=json.loads((p/'boundary_160.json').read_text());b=json.loads((source/'boundary.json').read_text())
 diff=np.linalg.norm(z['position_world_xy'][800:]-pred['position_world_xy'],axis=1);heading=np.abs(wrap(z['yaw_world_rad'][800:]-pred['yaw_world_rad']));aux=np.abs(z['joint_position_rad'][800:,1]-template['joint_position_rad'][400:,1]);main=np.abs(z['joint_velocity_rad_s'][800:,0]-template['joint_velocity_rad_s'][400:,0])
 differences={'aux_angle_deg':float(np.degrees(a['joint_pos'][1]-b['joint_pos'][1])),'joint_velocity_delta_rad_s':(np.array(a['joint_vel'])-b['joint_vel']).tolist(),'body_level_linear_velocity_delta_m_s':(levelvec(a['linear_velocity'],a['quaternion'])-levelvec(b['linear_velocity'],b['quaternion'])).tolist(),'body_level_angular_velocity_delta_rad_s':(levelvec(a['angular_velocity'],a['quaternion'])-levelvec(b['angular_velocity'],b['quaternion'])).tolist(),'roll_pitch_delta_deg':np.degrees(wrap(rp(a['quaternion'])-rp(b['quaternion']))).tolist(),'smoother_identical':a['smoother']==b['smoother'],'previous_action_identical':a['previous_action']==b['previous_action'],'full_recorded_state_identical':a==b}
 prefix_same={k:bool(np.array_equal(z[k][:801],prefix[k])) for k in prefix.files}
 accepted=bool(all(prefix_same.values()) and diff.max()<=.10 and heading.max()<=math.radians(5))
 out[kind]={'prefix_all_arrays_identical':all(prefix_same.values()),'prefix_per_array':prefix_same,'start_differences':differences,'max_xy_error_m':float(diff.max()),'final_xy_error_m':float(diff[-1]),'max_heading_error_deg':float(np.degrees(heading.max())),'max_aux_error_deg':float(np.degrees(aux.max())),'max_main_velocity_error_rad_s':float(main.max()),'accepted_010m_envelope':accepted,'hypothesis_scope':'two tested histories only; no arbitrary-state validity'}
 np.savez(h/f'{kind}_actual_third_transition.npz',time_s=z['simulation_time_s'][800:]-z['simulation_time_s'][800],position_world_xy=z['position_world_xy'][800:],yaw_world_rad=z['yaw_world_rad'][800:],joint_position_rad=z['joint_position_rad'][800:],joint_velocity_rad_s=z['joint_velocity_rad_s'][800:])
 actual=z['position_world_xy'][800:];o=actual[0]
 axes[row,0].plot(*(actual-o).T,label='actual continuous third turn');axes[row,0].plot(*(pred['position_world_xy']-o).T,'--',label='relocated isolated template');axes[row,0].axis('equal');axes[row,0].set_title(kind+' third transition');axes[row,0].set_xlabel('world delta X m');axes[row,0].set_ylabel('world delta Y m')
 axes[row,1].plot(np.arange(len(diff))*.02,diff,label='position error m');axes[row,1].axhline(.10,color='r',ls=':',label='frozen .10m gate');axes[row,1].set_xlabel('third turn time s')
 for ax in axes[row]:ax.legend();ax.grid(alpha=.3)
manifest=json.loads((h/'pre_run_identity.json').read_text());assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in manifest.items())
(h/'assessment.json').write_text(json.dumps(out,indent=2)+'\n');fig.tight_layout();fig.savefig(h/'comparison.png',dpi=150);print(json.dumps(out,indent=2))
