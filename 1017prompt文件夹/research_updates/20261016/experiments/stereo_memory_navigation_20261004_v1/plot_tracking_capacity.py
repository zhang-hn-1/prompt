"""Already measured actual diagnostics only; no new candidate or labels."""
from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;O=H/'endpoint_navigation_run'
def main():
 a=json.loads((H/'tracking_capacity_assessment.json').read_text());r=a['rows'];w=a['windows'];t=[x['time_s'] for x in r];tm=[(x['start_frame']+x['end_frame'])/20 for x in w]
 fig,axes=plt.subplots(2,2,figsize=(12,8))
 ax=axes[0,0];ax.plot(t,[x['nominal_body_curvature_1_m'] for x in r],label='Planned V1 body k');ax.plot(t,[x['actual_requested_body_curvature_1_m'] for x in r],label='Actual PP request k');ax.plot(t,[x['current_bank_weighted_endpoint_support_1_m'][1] for x in r],label='BankMap weighted upper endpoint');ax.set(ylabel='Body curvature descriptor (1/m)',title='Current control/model compatibility');ax.legend(fontsize=8)
 ax=axes[0,1];ax.plot(t,[x['desired_bank_mean_rad'] for x in r],label='Actual target bank');ax.plot(t,[x['actual_bank_mean_rad'] for x in r],label='Actual measured bank');ax.set(ylabel='Base body Y world-Z tilt (rad)',title='Early transition vs later target following');ax.legend(fontsize=8)
 ax=axes[1,0];ax.plot(tm,[x['xy_curvature_1_m'] for x in w],'o-',label='Actual three-point XY k');ax.plot(tm,[x['body_yaw_per_xy_arc_1_m'] for x in w],'s-',label='Body Euler yaw / actual XY arc');ax.plot(tm,[x['world_z_integral_per_xy_arc_1_m'] for x in w],'x-',label='World-Z integral / actual XY arc');ax.set(ylabel='Different measured descriptors (1/m)',title='All ten windows retained, no stable-motion claim');ax.legend(fontsize=8)
 ax=axes[1,1];ax.plot(t,[x['current_projection_distance_m'] for x in r],label='Own known-reference waypoint distance');ax.plot(t,[x['current_signed_body_heading_error_rad'] for x in r],label='Current profile heading error (rad)');ax.set(ylabel='Position (m) / heading (rad)',title='Current tracking diagnostics, distinct units');ax.legend(fontsize=8)
 for ax in axes.flat:ax.set_xlabel('Actual simulation time (s)');ax.grid(alpha=.25)
 fig.tight_layout();fig.savefig(H/'tracking_capacity_measurements.png',dpi=150);plt.close(fig)
 xy=np.load(O/'episode_arrays.npz')['positions_world_xy'];ref=np.load(O/'local_plans/frame_334.npz')['pose'][:,:2];g=a['guard_counterfactuals'][-1]['guard'];witness=a['raster_witnesses'][0];grid=np.load(O/'maps/frame_335.npz');haz=grid['origin']+np.argwhere(grid['labels']==1)*float(grid['resolution'])
 fig,ax=plt.subplots(figsize=(7,6));ax.scatter(haz[:,0],haz[:,1],s=6,c='grey',label='Actual observed hazard centres');ax.plot(ref[:,0],ref[:,1],'k--',label='Known reference3');ax.plot(xy[237:336,0],xy[237:336,1],label='Actual forward trajectory');ax.plot(*np.array(g['bridge_xy']).T,'tab:orange',label='Actual335 projection bridge');ax.scatter(*witness['sample_world_xy'],c='green',marker='x',s=100,label='Continuous projection: .640679m');ax.scatter(*witness['rounded_world_xy'],c='red',marker='s',s=70,label='Rounded cell: .583095m');ax.scatter(*witness['hazard_world_xy'],c='purple',s=80,label='Hazard from actual frame131');ax.set(xlim=(-1.3,1.3),ylim=(-3.4,-1.5),xlabel='World X (m)',ylabel='World Y (m)',title='335 guard: actual position changes screen; map swap does not');ax.axis('equal');ax.grid(alpha=.25);ax.legend(fontsize=8);fig.tight_layout();fig.savefig(H/'tracking_capacity_guard_witness.png',dpi=150);plt.close(fig)
if __name__=='__main__':main()
