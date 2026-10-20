"""Actual clean runs; map is an offline illustration only."""
import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
HERE=Path(__file__).parent
PROJECT=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')


def main():
    results=json.loads((HERE/'legacy_clean_assessment.json').read_text())['results']
    fig,axes=plt.subplots(1,3,figsize=(15,5),constrained_layout=True)
    for ax,label in zip(axes,('e0','e1','e2')):
        root=HERE/'legacy_clean'/label;folder=root/f'episode_golden_{label}_0001'
        spec=next(e for e in json.loads((folder/'manifest.json').read_text())['episodes'] if e['episode_id']==f'golden_{label}_0001')
        if label in ('e1','e2'):
            name='e1_detour_bodyband.npz' if label=='e1' else 'e2_corridor_bodyband.npz'
            with np.load(PROJECT/'data/benchmark_v1/occupancy'/name) as f:g,o,res=f['grid'],f['global_origin_xy'],float(f['resolution_m'])
            ax.imshow(g.T,origin='lower',extent=(o[0]-res/2,o[0]+(g.shape[0]-.5)*res,o[1]-res/2,o[1]+(g.shape[1]-.5)*res),
                cmap=ListedColormap(['#fafafa','#777','#ddd']),vmin=0,vmax=2)
        with np.load(folder/'states.npz') as f:xy=f['position_world_xy']
        ax.plot(xy[:,0],xy[:,1],lw=2.5,c='#247c65',label='actual clean-render RGB policy')
        if (root/'vision_reference.npz').exists():
            with np.load(root/'vision_reference.npz') as f:path=f['position_world_xy']
            ax.plot(path[:,0],path[:,1],'--',lw=1,c='#497dbb',label='left RGB-triggered local curve')
        goal=np.asarray(spec['goal_position_world_m'][:2]);ax.scatter(*xy[0],c='black',s=25,label='start')
        ax.scatter(*goal,c='#c34646',marker='*',s=100,label='goal')
        ax.set_xlim(xy[:,0].min()-.7,xy[:,0].max()+.7);ax.set_ylim(min(xy[:,1].min(),-1)-.5,max(xy[:,1].max(),1)+.5)
        ax.set_aspect('equal');ax.set_xlabel('world X (m)');ax.set_ylabel('world Y (m)')
        ax.set_title(f"{label.upper()}: success {results[label]['time_s']:.1f}s, no contact")
        ax.legend(fontsize=7,loc='best')
    fig.savefig(HERE/'legacy_clean_e0_e1_e2_actual_trajectories.png',dpi=150)


if __name__=='__main__':main()
