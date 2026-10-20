"""Exploratory offline audit of the frozen RGB classifier; no controller changes."""
import sys,json
from pathlib import Path
import numpy as np,torch,cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).parent;E=H.parent
sys.path.insert(0,str(E/'fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1'))
from policy import pilot,MODEL
sys.path.insert(0,str(E/'bilateral_temporal_geometry_navigation_20261003_v1'))
from geometry import camera_pose,project
masks=np.load(E/'fixed_rig_self_mask_and_curve_qualification_20261003_v1/base_link_masks.npz')
torch.set_num_threads(2);model=pilot.TinyPixelNet().eval();model.load_state_dict(torch.load(MODEL,map_location='cpu',weights_only=True))
runs={**{s:E/'fixed_rig_self_mask_and_curve_qualification_20261003_v1/legacy_clean'/s for s in ('e0','e1','e2')},**{s:H/'baseline'/s for s in ('hard_0_e1_0003','hard_0_e1_0004')}}
xs=np.arange(.6,6.01,.1);ys=np.arange(-3.,3.01,.1);x,y=np.meshgrid(xs,ys);local=np.column_stack((x.ravel(),y.ravel()))
fig,axs=plt.subplots(5,3,figsize=(14,18));summary={}
for row,(name,root) in enumerate(runs.items()):
 with np.load(root/'online_inputs/frame_000.npz') as f,np.load(root/'offline_truth/frame_000.npz') as truth:
  xy=f['position_world_xy'];yaw=float(f['yaw_world_rad']);q=f['quaternion_world_wxyz_history'][-1];c,s=np.cos(yaw),np.sin(yaw)
  world=np.column_stack((xy+local@np.array([[c,s],[-s,c]]),np.zeros(len(local))))
  summary[name]={};pc=[]
  for e,eye in enumerate(('left','right')):
   with torch.no_grad():prob=torch.softmax(model(pilot.rgb_input(f[eye+'_rgb'][::4,::4][None],np.array([e]),torch.device('cpu'))),1).numpy()[0];pred=prob.argmax(0)
   uv,z=project(world,camera_pose(xy,q,eye));ij=np.rint(uv).astype(int);valid=(z>0)&(ij[:,0]>=0)&(ij[:,0]<640)&(ij[:,1]>=0)&(ij[:,1]<480);ii=np.clip(ij[:,0],0,639);jj=np.clip(ij[:,1],0,479);valid&=~masks[eye][jj,ii]
   labels=np.full(len(local),2);labels[valid]=pred[jj[valid]//4,ii[valid]//4]
   # Compare class predicted at nominal ground points to independent depth's hit height.
   depth=truth[eye+'_depth_m'][jj,ii];pose=truth[eye+'_camera_usd_world'];rot=np.column_stack((pose[:3,0],-pose[:3,1],-pose[:3,2]));rays=np.column_stack(((ii-pilot.CX)/pilot.FOCAL_PX,(jj-pilot.CY)/pilot.FOCAL_PX,np.ones(len(ii))));hits=pose[:3,3]+rays*depth[:,None]@rot.T
   central=valid&(np.abs(local[:,1])<=.4)&(local[:,0]<=4)
   summary[name][eye]={'central_visible':int(central.sum()),'central_rgb_obstacle':int((central&(labels==1)).sum()),'central_true_high':int((central&(hits[:,2]>.25)&(hits[:,2]<1.5)).sum())}
   pc.append((labels,valid));axs[row,e].imshow(f[eye+'_rgb']);overlay=np.ma.masked_where((pred!=1)|masks[eye][::4,::4],pred);axs[row,e].imshow(overlay,extent=(0,640,480,0),alpha=.5,cmap='autumn',vmin=0,vmax=1);axs[row,e].set_title(name+' '+eye);axs[row,e].axis('off')
  lab=np.full(len(local),2);lab[(pc[0][1]&(pc[0][0]==0))|(pc[1][1]&(pc[1][0]==0))]=0;lab[(pc[0][1]&(pc[0][0]==1))|(pc[1][1]&(pc[1][0]==1))]=1
  axs[row,2].scatter(local[:,0],local[:,1],c=np.array(['lightgreen','red','silver'])[lab],s=7);axs[row,2].axhline(.4,ls='--');axs[row,2].axhline(-.4,ls='--');axs[row,2].set_aspect('equal');axs[row,2].set_title('ground projection: red RGB obstacle, gray unknown');axs[row,2].set_xlabel('forward m');axs[row,2].set_ylabel('left m')
fig.tight_layout();fig.savefig(H/'ground_projection_exploration.png',dpi=120);(H/'ground_projection_exploration.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
