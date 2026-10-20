from pathlib import Path
import json,numpy as np,cv2,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;C=H/'original_E1_static_sparse_memory_navigation_run_v1';B=H/'original_E1_direct_memory_navigation_run_v1';O=H/'original_E1_static_sparse_AB_visuals_v1';O.mkdir(exist_ok=False)
fig,axes=plt.subplots(1,3,figsize=(14,4));goal=np.array([3.300000005960465,5.960465e-9])
shapes=json.loads((H/'original_E1_static_sparse_membership_v1/scene_collision_shapes.json').read_text())
for c in shapes:axes[0].add_patch(plt.Circle(c['position'][:2],c['radius'],color='gray',alpha=.35))
for name,D in [('B0 TTL10 incomplete60, actual54.6',B),('C static past-measured risk',C)]:
 z=np.load(D/'physics.npz');p=z['position_world_xy'];t=z['simulation_time_s'];axes[0].plot(p[:,0],p[:,1],label=name);axes[1].plot(t,np.linalg.norm(p-goal,axis=1),label=name);axes[2].plot(t,p[:,0]-p[0,0],label=name)
axes[0].scatter(*goal,label='goal');axes[0].set_aspect('equal');axes[0].set_xlim(-4,4);axes[0].set_ylim(-3,2);axes[0].set_xlabel('World X (m)');axes[0].set_ylabel('World Y (m)');axes[0].legend(fontsize=7);axes[0].set_title('Original E1, knownpose TaskH only')
axes[1].set_title('True goal distance (m)');axes[2].set_title('True forward-axis progress (m)')
for ax in axes[1:]:ax.set_xlabel('Simulation time (s)');ax.legend(fontsize=7)
fig.tight_layout();fig.savefig(O/'true_progress_and_trajectory.png',dpi=150);plt.close(fig)
cap=cv2.VideoCapture(str(C/'stereo_view.mp4'));N=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));chosen=sorted(set([0,min(60,N-1),int((N-1)*.4),int((N-1)*.6),int((N-1)*.8),N-1]));images=[]
for i in chosen:
 cap.set(cv2.CAP_PROP_POS_FRAMES,i);ok,im=cap.read();assert ok;im=cv2.resize(im,(640,240));cv2.putText(im,f'Original E1 C knownpose t={i/10:.1f}s',(10,20),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,255),1);images.append(im)
cap.release()
while len(images)<6:images.append(images[-1])
cv2.imwrite(str(O/'stereo_contact_sheet.png'),np.vstack([np.hstack(images[:3]),np.hstack(images[3:6])]))
print('visuals saved',O)
