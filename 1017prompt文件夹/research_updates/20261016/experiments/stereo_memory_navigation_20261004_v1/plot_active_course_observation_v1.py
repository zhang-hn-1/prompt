from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;D=H/'active_course_observation_run_v1';A=H/'active_course_observation_causal_audit_v1'
a=json.loads((H/'active_course_observation_assessment_v1.json').read_text());v=json.loads((A/'mutual_visibility_audit_v1.json').read_text())['rows'];states=json.loads((D/'states.json').read_text());fig,axs=plt.subplots(2,2,figsize=(13,9));ph=np.load(D/'physics.npz')
for folder,label in [(H/'qualified_hazard_navigation_run_v1','Historical source consumer'),(D,'Active-course observation')]:
 x=np.load(folder/'physics.npz')['root_position_world_m'];axs[0,0].plot(x[:,0],x[:,1],label=label);axs[0,0].scatter(*x[-1,:2],marker='x')
truth=np.load(D/'offline_occupancy_label.npz');occ=truth['origin']+np.argwhere(truth['labels']>0)*float(truth['resolution']);axs[0,0].scatter(occ[:,0],occ[:,1],s=2,c='.75',label='Offline obstacle proxy');axs[0,0].set_aspect('equal');axs[0,0].set_xlabel('World X [m]');axs[0,0].set_ylabel('World Y [m]');axs[0,0].set_title('Actual paths; historical comparison only');axs[0,0].legend(fontsize=8)
t=[r['time_s'] for r in v];axs[0,1].plot(t,[r['same_surface_bilateral_visible'] for r in v],label='True common visible hazard pixels');axs[0,1].plot(t,[r['mutual_visible_model_confident_in_range'] for r in v],label='Fast confidence + range on those pixels');axs[0,1].set_yscale('symlog',linthresh=1);axs[0,1].set_xlabel('Time [s]');axs[0,1].set_title('Availability versus source output (offline diagnostic)');axs[0,1].legend(fontsize=8)
r=a['all_window_source_updates'];axs[1,0].plot([x['time_s'] for x in r],[x['n'] for x in r],'-o',label='Emitted hazard points');axs[1,0].plot([x['time_s'] for x in r],[x['correct'] for x in r],'-o',label='Both-endpoint correct');axs[1,0].set_title('All observation source updates');axs[1,0].set_xlabel('Time [s]');axs[1,0].legend(fontsize=8)
b=a['bank_public_gravity_trace'];axs[1,1].plot([x['time_s'] for x in b],np.rad2deg([x['public_gravity_bank_rad'] for x in b]),label='Measured gravity bank [degrees]');elev=[]
for x in b:
 T=np.load(D/'known_poses'/f"frame_{x['frame']:03d}.npz")['left_camera_usd_world'];elev.append(np.rad2deg(np.arcsin(np.clip(-T[2,2],-1,1))))
axs[1,1].plot([x['time_s'] for x in b],elev,label='Optical-axis elevation [degrees]');axs[1,1].set_title('Measured bank and camera-axis elevation');axs[1,1].set_xlabel('Time [s]');axs[1,1].legend(fontsize=8)
for ax in [axs[0,1],axs[1,0],axs[1,1]]:ax.axvspan(a['event_time_s'],a['probe_end_time_s'],color='green',alpha=.1)
fig.tight_layout();out=A/'actual_visibility_and_bank_v1.png';assert not out.exists();fig.savefig(out,dpi=150);print(out)
# Native stereo snapshots deliberately preserve original orientation and mounting.
import cv2
fig,axs=plt.subplots(3,2,figsize=(12,12))
snapshot_frames=[min(int(a['event_time_s']*10),len(states)-1),min(int((a['event_time_s']+3)*10),len(states)-1),min(int((a['event_time_s']+6)*10),len(states)-1)]
for row,i in enumerate(snapshot_frames):
 rgb=np.load(D/'online_inputs'/f'frame_{i:03d}.npz')
 for col,eye in enumerate(['left','right']):axs[row,col].imshow(rgb[eye+'_rgb']);axs[row,col].set_title(f'Native {eye}, frame {i}, {states[i]["time_s"]:.1f}s');axs[row,col].axis('off')
fig.tight_layout();out=A/'actual_native_stereo_snapshots_v1.png';assert not out.exists();fig.savefig(out,dpi=130);print(out)
