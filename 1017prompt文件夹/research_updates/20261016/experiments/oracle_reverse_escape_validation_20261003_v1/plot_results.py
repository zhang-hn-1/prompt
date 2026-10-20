"""Actual trajectories and measured terminal chain, never policy index scores."""
import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

HERE = Path(__file__).resolve().parent
PROJECT = Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')


def main():
    assessment = json.loads((HERE/'assessment.json').read_text())
    with np.load(PROJECT/'data/benchmark_v1/occupancy/cluttered_hard_0_bodyband.npz') as a:
        grid, origin, res = a['grid'], a['global_origin_xy'], float(a['resolution_m'])
    extent = (origin[0]-res/2,origin[0]+(grid.shape[0]-.5)*res,
              origin[1]-res/2,origin[1]+(grid.shape[1]-.5)*res)
    fig, axes = plt.subplots(2,2,figsize=(12,10),constrained_layout=True)
    for ax,result in zip(axes.flat,assessment['results']):
        episode = result['episode_id'];root = HERE/episode;folder=root/f'episode_{episode}'
        spec = next(e for e in json.loads((folder/'manifest.json').read_text())['episodes'] if e['episode_id']==episode)
        goal = np.asarray(spec['goal_position_world_m'][:2])
        with np.load(folder/'states.npz') as a:
            xy,t=a['position_world_xy'],a['simulation_time_s']
        ax.imshow(grid.T,origin='lower',extent=extent,cmap=ListedColormap(['#fafafa','#606060','#ddd']),vmin=0,vmax=2)
        if result['route_built']:
            with np.load(root/'reference_built_at_6s.npz') as a:
                path=a['position_world_xy']
            ax.plot(path[:,0],path[:,1],'--',color='#4683bf',lw=1,label='actual Oracle reference')
        prefix=t<=6.00001
        ax.plot(xy[prefix,0],xy[prefix,1],color='#e29527',lw=2,label='same fixed escape')
        ax.plot(xy[~prefix,0],xy[~prefix,1],color='#22964b',lw=2,label='reverse tracking / settling')
        ax.scatter(*xy[0],c='black',s=35,label='start',zorder=5)
        ax.scatter(*goal,c='#2d65b2',marker='*',s=130,label='goal',zorder=5)
        ax.scatter(*xy[-1],c='#d43b35' if result['contact_detected'] else '#22964b',marker='x',s=70,zorder=5)
        envelope=np.vstack((xy,goal));ax.set_xlim(envelope[:,0].min()-.9,envelope[:,0].max()+.9)
        ax.set_ylim(envelope[:,1].min()-.9,envelope[:,1].max()+.9)
        ax.set_aspect('equal');ax.set_xlabel('world X (m)');ax.set_ylabel('world Y (m)')
        if result['formal_goal_entry_success']:
            outcome=f"goal {result['first_goal_time_s']:.1f}s, stop FAILED"
            if result['contact_detected']:
                outcome+=f"; contact {result['first_contact_s']:.2f}s"
        else:
            outcome=f"collision {result['first_contact_s']:.2f}s ({result['final_policy_stage']})"
        ax.set_title(f'{episode}: {outcome}');ax.legend(fontsize=7.3)
    fig.savefig(HERE/'four_episode_actual_trajectories.png',dpi=160);plt.close(fig)
    root=HERE/'hard_0_e1_0004';folder=root/'episode_hard_0_e1_0004'
    spec=next(e for e in json.loads((folder/'manifest.json').read_text())['episodes'] if e['episode_id']=='hard_0_e1_0004')
    goal=np.asarray(spec['goal_position_world_m'][:2])
    with np.load(folder/'states.npz') as a:
        xy,t=a['position_world_xy'],a['simulation_time_s']
        speed=np.linalg.norm(a['linear_velocity_world_m_s'][:,:2],axis=1)
    with np.load(root/'physics.npz') as a:
        pt=a['simulation_time_s'];raw=a['raw_command_target'][:,0];smooth=a['smoothed_target'][:,0]
        applied=a['applied_action'][:,0];joint=a['joint_velocity_rad_s'][:,0]
    fig, axes=plt.subplots(1,3,figsize=(15,4.8),constrained_layout=True)
    idx=t>=26;axes[0].plot(xy[idx,0],xy[idx,1],color='#29974e')
    axes[0].add_patch(plt.Circle(goal,.2,color='#447ebb',alpha=.2))
    axes[0].scatter(*goal,marker='*',c='#447ebb',s=100,label='goal + 0.20 m radius')
    e=int(np.flatnonzero(np.linalg.norm(xy-goal,axis=1)<=.2)[0])
    axes[0].scatter(*xy[e],c='#e29527',label='28.4 s: first entry',zorder=5)
    axes[0].scatter(*xy[-1],c='#d43b35',marker='x',label='31.4 s: 1.22 m away',zorder=5)
    axes[0].set_aspect('equal');axes[0].set_xlabel('world X (m)');axes[0].set_ylabel('world Y (m)')
    axes[0].legend(fontsize=8);axes[0].set_title('Measured overshoot after zero request')
    axes[1].plot(t,np.linalg.norm(xy-goal,axis=1),color='#29974e',label='distance to goal (m)')
    axes[1].plot(t,speed,color='#447ebb',label='world XY speed (m/s)')
    axes[1].axhline(.2,c='#29974e',ls=':');axes[1].axhline(.05,c='#447ebb',ls=':')
    axes[1].set_ylim(0,1.5);axes[1].legend(fontsize=8);axes[1].set_title('Goal entry does not imply stop')
    axes[2].plot(pt,raw,label='raw main request',color='#888')
    axes[2].plot(pt,smooth,label='smoothed main target',color='#ed9028')
    axes[2].plot(pt,applied,label='applied main target',color='#29974e',ls='--')
    axes[2].plot(pt,joint,label='measured main joint velocity',color='#447ebb',alpha=.7)
    axes[2].set_ylabel('rad/s');axes[2].legend(fontsize=8);axes[2].set_title('Execution retains -1.70 rad/s at entry')
    for ax in axes[1:]:
        ax.set_xlim(26,31.5);ax.axvline(28.4,color='#d43b35',ls=':');ax.set_xlabel('simulation time (s)');ax.grid(alpha=.2)
    fig.savefig(HERE/'goal_entry_and_stop_chain.png',dpi=170);plt.close(fig)
    print(HERE/'four_episode_actual_trajectories.png');print(HERE/'goal_entry_and_stop_chain.png')


if __name__=='__main__':
    main()
