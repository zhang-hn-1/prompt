from pathlib import Path
import json,numpy as np,cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
H=Path(__file__).resolve().parent;D=H/'original_E1_repeated_exposure_memory_navigation_run_v1';O=H/'E1_repeated_exposure_memory_visuals_v1';O.mkdir(exist_ok=False)
z=np.load(D/'physics.npz');old=np.load(H/'original_E1_nonbinding_side_hypothesis_navigation_run_v1/physics.npz');shapes=json.loads((H/'original_E1_route_side_resumed_sparse_membership_v2/scene_collision_shapes.json').read_text());rows=json.loads((D/'policy_records.json').read_text());t=z['simulation_time_s'];xy=z['position_world_xy'];goal=np.array([3.300000005960465,5.960465010446114e-9]);fig,axs=plt.subplots(2,2,figsize=(13,10));a=axs[0,0]
for c in shapes:a.add_patch(Circle(c['position'][:2],c['radius'],color='gray',alpha=.4))
a.plot(old['position_world_xy'][:,0],old['position_world_xy'][:,1],label='Historical nonbinding baseline (different RGB source)',alpha=.7);a.plot(xy[:,0],xy[:,1],label='Repeated-exposure memory fresh actual');a.scatter(*goal,c='green',marker='*',s=120,label='Goal');a.scatter(*xy[-1],c='red',marker='x',s=80,label='54.6s stop');a.set(xlim=(-3.8,3.8),ylim=(-3.5,4),xlabel='World x (m)',ylabel='World y (m)',title='Known-pose TaskH: actual trajectory, not original35');a.set_aspect('equal');a.legend(fontsize=8)
axs[0,1].plot(t,np.linalg.norm(xy-goal,axis=1),label='True goal distance');axs[0,1].axhline(4.182900500,ls='--',label='54.6s maximum allowed');axs[0,1].set(xlabel='Simulation time (s)',ylabel='m',title='Failed frozen progress gate');axs[0,1].legend()
requests=np.load(D/'requests.npz')['requests'];rt=np.arange(len(requests))*.1;axs[1,0].plot(rt,requests[:,0],label='Requested main (rad/s)');axs[1,0].plot(rt,requests[:,1],label='Requested aux (rad)');axs[1,0].set(xlabel='Simulation time (s)',title='Actual requests, pending reversal and failed replanning');axs[1,0].legend(fontsize=8)
axs[1,1].plot(t,xy[:,0]-(-3.2999999940395353),label='True axis progress');axs[1,1].axhline(2.427377623,ls='--',label='54.6s minimum required');axs[1,1].set(xlabel='Simulation time (s)',ylabel='m',title='Path index is not progress');axs[1,1].legend()
fig.tight_layout();fig.savefig(O/'trajectory_progress_and_requests.png',dpi=150);plt.close(fig)
cap=cv2.VideoCapture(str(D/'third_person.mp4'));tiles=[]
for i in [0,155,180,224,285,350,355,546]:
 cap.set(cv2.CAP_PROP_POS_FRAMES,i);ok,img=cap.read();assert ok;img=cv2.resize(img,(480,270));cv2.putText(img,f'Actual frame {i} / {i/10:.1f}s',(10,25),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),2);tiles.append(img)
cap.release();cv2.imwrite(str(O/'actual_third_person_frames.png'),np.vstack([np.hstack(tiles[j:j+2]) for j in range(0,8,2)]));print(O)
