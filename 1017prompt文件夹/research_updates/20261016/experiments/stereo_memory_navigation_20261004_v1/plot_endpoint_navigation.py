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
 assessment=json.loads((H/'endpoint_navigation_assessment.json').read_text())
 fig,axes=plt.subplots(1,3,figsize=(15,4.5));ref=np.load(H/'oracle_rgb_shadow_reference.npz')['pose'][:,:2]
 axes[0].plot(ref[:,0],ref[:,1],'k--',label='Independent evaluation reference')
 for name,folder,color in [('Guarded B0 (archived)','guarded_reference_run','tab:blue'),('Root cost (rejected comparator)','root_direction_run','tab:green'),('Cusp projection (rejected)','cusp_navigation_run','tab:orange'),('Endpoint query candidate','endpoint_navigation_run','tab:red')]:
  a=np.load(H/folder/'episode_arrays.npz');xy=a['positions_world_xy'];s,e,_=project(xy,ref);t=np.arange(len(xy))/10.
  axes[0].plot(xy[:,0],xy[:,1],color=color,label=name);axes[0].scatter(*xy[-1],color=color,marker='x')
  axes[1].plot(t,s,color=color,label=name);axes[2].plot(t,e,color=color,label=name)
 axes[0].scatter(*ref[-1],marker='*',s=130,c='green',label='Goal');axes[0].set(xlabel='World X (m)',ylabel='World Y (m)',title='Actual trajectories, same initial condition');axes[0].axis('equal')
 axes[1].set(xlabel='Simulation time (s)',ylabel='Actual projected progress (m)',title='Frozen evaluator, independent of path index')
 axes[2].set(xlabel='Simulation time (s)',ylabel='Distance to frozen reference (m)',title='Actual XY lateral error')
 for ax in axes:ax.grid(alpha=.25);ax.legend(fontsize=8)
 fig.suptitle('ONE35s outgoing endpoint query: navigation '+('PASS' if assessment['navigation_gate_pass'] else 'REJECTED'))
 fig.tight_layout();fig.savefig(H/'endpoint_navigation_comparison.png',dpi=160);plt.close(fig)

if __name__=='__main__':main()
