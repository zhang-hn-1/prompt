"""Actual A/B tracks and the RGB-built candidates; map is offline backdrop."""
import json
import os
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from control import make_detour

HERE=Path(os.environ.get('BILATERAL_ASSESS_ROOT',Path(__file__).resolve().parent))
PROJECT=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')


def main():
    assessment=json.loads((HERE/'navigation_assessment.json').read_text())
    fig,axes=plt.subplots(2,2,figsize=(13,10),constrained_layout=True)
    for column,label in enumerate(('e2','e1')):
        ax=axes[0,column];ep=f'episode_golden_{label}_0001'
        occupancy={'e1':'e1_detour_bodyband.npz','e2':'e2_corridor_bodyband.npz'}[label]
        with np.load(PROJECT/'data/benchmark_v1/occupancy'/occupancy) as f:
            grid,origin,res=f['grid'],f['global_origin_xy'],float(f['resolution_m'])
        extent=(origin[0]-res/2,origin[0]+(grid.shape[0]-.5)*res,origin[1]-res/2,origin[1]+(grid.shape[1]-.5)*res)
        ax.imshow(grid.T,origin='lower',extent=extent,cmap=ListedColormap(['#fafafa','#666','#ddd']),vmin=0,vmax=2)
        candidate=HERE/f'candidate_{label}'
        with np.load(candidate/'vision_reference.npz') as f:path=f['position_world_xy']
        ax.plot(path[:,0],path[:,1],'--',c='#399a65',lw=1.2,label='RGB-triggered proposed curve')
        trajectories=[]
        for mode,color in [('b0','#497dbb'),('candidate','#d8533d')]:
            with np.load(HERE/f'{mode}_{label}'/ep/'states.npz') as f:
                xy=f['position_world_xy'];trajectories.append(xy)
            result=assessment['results'][f'{mode}_{label}']
            name='no-vision B0' if mode=='b0' else 'bilateral RGB candidate'
            ax.plot(xy[:,0],xy[:,1],c=color,lw=2.3,label=f"{name}: {result['termination']} {result['elapsed_s']:.1f}s")
            if result['contact']:ax.scatter(*xy[-1],c=color,marker='x',s=70,lw=2,zorder=5)
        manifest=json.loads((candidate/ep/'manifest.json').read_text())
        spec=next(e for e in manifest['episodes'] if e['episode_id']==f'golden_{label}_0001')
        goal=np.asarray(spec['goal_position_world_m'][:2]);ax.scatter(*goal,c='#4776bb',marker='*',s=150,label='goal')
        ax.scatter(*trajectories[0][0],c='black',s=35,label='same initial physical state')
        envelope=np.vstack((path,*trajectories,goal))
        ax.set_xlim(envelope[:,0].min()-.65,envelope[:,0].max()+.65)
        ax.set_ylim(envelope[:,1].min()-.65,envelope[:,1].max()+.65)
        ax.set_aspect('equal');ax.set_xlabel('world X (m)');ax.set_ylabel('world Y (m)');ax.legend(fontsize=7.6)
        ax.set_title(f"{label.upper()}: actual closed-loop outcome (35 s rule)")
        records=json.loads((candidate/'vision_records.json').read_text());event=next(r for r in records if r['reference_created'])
        with np.load(candidate/'online_inputs'/f"frame_{event['block']:03d}.npz") as f:
            points=f['observed_obstacle_xy'];pos=f['position_world_xy'];yaw=float(f['yaw_world_rad']);goalbase=f['pointgoal_base_xy']
        c,s=np.cos(yaw),np.sin(yaw);gw=pos+np.array([[c,-s],[s,c]])@goalbase
        forward=(gw-pos)/np.linalg.norm(gw-pos);left=np.array([-forward[1],forward[0]])
        transform=np.column_stack((forward,left));cloud=(points-pos)@transform
        ax=axes[1,column];ax.set_facecolor('#eee')
        ax.scatter(cloud[:,0],cloud[:,1],s=24,c='#d8533d',label='past/current RGB-inferred obstacle cells')
        for side,color in [(-1,'#be792e'),(1,'#5b79b8')]:
            route=make_detour(pos,gw,side);local=(route-pos)@transform;chosen=event['side']==side
            ax.plot(local[:,0],local[:,1],color=color,lw=2.3 if chosen else 1,ls='-' if chosen else '--',
                    label=f"{'left' if side==1 else 'right'} proposal{' (chosen)' if chosen else ''}")
        ax.axhspan(-.6,.6,color='white',alpha=.5,zorder=0);ax.scatter(0,0,c='black',s=35,label='current position')
        ax.set_xlim(-.5,4);ax.set_ylim(-3,3);ax.set_aspect('equal');ax.grid(alpha=.2)
        ax.set_xlabel('toward PointGoal (m)');ax.set_ylabel('left of PointGoal axis (m)')
        ax.set_title(f"{label.upper()}: RGB geometry at {event['time_s']:.1f}s\nGray = unobserved; proposals are unverified",fontsize=11)
        ax.legend(fontsize=7.4,loc='upper right')
    fig.savefig(HERE/'actual_visual_navigation_and_local_curves.png',dpi=160);plt.close(fig)
    # Both original eye images at each actual trigger, without truth overlays.
    fig,axes=plt.subplots(2,2,figsize=(12,9),constrained_layout=True)
    for row,label in enumerate(('e2','e1')):
        root=HERE/f'candidate_{label}';event=next(r for r in json.loads((root/'vision_records.json').read_text()) if r['reference_created'])
        with np.load(root/'online_inputs'/f"frame_{event['block']:03d}.npz") as f:
            for col,eye in enumerate(('left','right')):
                axes[row,col].imshow(f[f'{eye}_rgb']);axes[row,col].axis('off')
                uv=f[f'{eye}_uv'];axes[row,col].scatter(uv[:,0],uv[:,1],s=18,facecolors='none',edgecolors='#f0c938')
                axes[row,col].set_title(f"{label.upper()} {event['time_s']:.1f}s {eye}: {len(uv)} current accepted depth points")
    fig.savefig(HERE/'both_eye_images_at_actual_trigger.png',dpi=160);plt.close(fig)
    print(HERE/'actual_visual_navigation_and_local_curves.png')


if __name__=='__main__':main()
