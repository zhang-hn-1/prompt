"""Saved actual trajectories and fixed evaluator; visualization only."""
from pathlib import Path
import json
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from route_geometry_audit import project
H=Path(__file__).resolve().parent

def main():
 assessment=json.loads((H/'cusp_navigation_assessment.json').read_text())
 fig,axes=plt.subplots(1,3,figsize=(15,4.5));ref=np.load(H/'oracle_rgb_shadow_reference.npz')['pose'][:,:2]
 axes[0].plot(ref[:,0],ref[:,1],'k--',label='Independent evaluation reference')
 for name,folder,color in [('Guarded B0','guarded_reference_run','tab:blue'),('Root-direction candidate','cusp_navigation_run','tab:orange')]:
  a=np.load(H/folder/'episode_arrays.npz');xy=a['positions_world_xy'];s,e,_=project(xy,ref);t=np.arange(len(xy))/10.
  axes[0].plot(xy[:,0],xy[:,1],color=color,label=name);axes[0].scatter(*xy[-1],color=color,marker='x')
  axes[1].plot(t,s,color=color,label=name);axes[2].plot(t,e,color=color,label=name)
 axes[0].scatter(*ref[-1],marker='*',s=130,c='green',label='Goal');axes[0].set(xlabel='World X (m)',ylabel='World Y (m)',title='Actual trajectories, same initial condition');axes[0].axis('equal')
 axes[1].set(xlabel='Simulation time (s)',ylabel='Actual projected progress (m)',title='Frozen evaluator, independent of path index')
 axes[2].set(xlabel='Simulation time (s)',ylabel='Distance to frozen reference (m)',title='Actual XY lateral error')
 for ax in axes:ax.grid(alpha=.25);ax.legend(fontsize=8)
 fig.suptitle('ONE35s root-direction consumer: navigation '+('PASS' if assessment['navigation_gate_pass'] else 'REJECTED'))
 fig.tight_layout();fig.savefig(H/'cusp_navigation_comparison.png',dpi=160);plt.close(fig)
 # Actual recorded views, one fixed mid-navigation capture and final capture. Never candidate reference renders.
 images=[];out=H/'cusp_navigation_run'
 for name in ['stereo_view','third_person']:
  cap=cv2.VideoCapture(str(out/(name+'.mp4')));count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));slots=[]
  for index in [175,count-1]:
   cap.set(cv2.CAP_PROP_POS_FRAMES,index);ok,frame=cap.read();assert ok
   scale=960/frame.shape[1];frame=cv2.resize(frame,(960,int(frame.shape[0]*scale)))
   cv2.putText(frame,f'{name}: frame {index}, t={index/10:.1f}s',(12,28),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),2);slots.append(frame)
  cap.release();cv2.imwrite(str(H/('cusp_navigation_'+name+'_frames.png')),np.vstack(slots))
if __name__=='__main__':main()
