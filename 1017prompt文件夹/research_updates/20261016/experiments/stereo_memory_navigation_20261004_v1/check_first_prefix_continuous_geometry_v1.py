from pathlib import Path
import json,sys,numpy as np
H=Path(__file__).resolve().parent;D=H/'E1_continuous_tube_navigation_run_v1';sys.path.insert(0,'/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from continuous_cell_tube_v1 import continuous_cell_tube_screen
x=json.loads((D/'first_primitive_blockage_audit_v2.json').read_text());scores=[]
for r in x['states'][:2]:
 m=np.load(D/'maps'/f"frame_{r['map_frame']:03d}.npz");g=RepeatedExposureMemoryGrid(m['origin'],10,static_scene_assumption=True);g.labels=m['labels'].copy();comp=[]
 for trace in r['trace']:
  q=np.asarray(trace['samples'])[:,:2];v=continuous_cell_tube_screen(g,dict(bridge_xy=q[:1],suffix_xy=q));comp.append(dict(expansion=trace['expansion'],main=trace['main'],aux=trace['aux'],original_sampled_inflation_pass=trace['passed'],continuous_nominal_cell_tube_pass=v['pass'],continuous_clearance_m=v['minimum_continuous_cell_clearance_m']))
 scores.append(dict(frame=r['frame'],map_frame=r['map_frame'],status=r['status'],expanded=r['expanded_nodes'],comparisons=comp))
out=dict(scope='POSTCLOSE necessary geometric comparison on same sampled nominal .3m arcs converted to line segments. Not exact curved swept volume or transient physical prediction, no change to planner/actor/margin. New point removability not licensed.',states=scores,old_margin_unchanged=True,formal35_NOTRUN=True);(D/'first_prefix_continuous_geometry_v1.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(scores))
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=x['states'][0];m=np.load(D/'maps/frame_506.npz');risk=g.origin+np.argwhere(m['labels']==1)*g.resolution
fig,ax=plt.subplots(figsize=(8,7));ax.scatter(risk[:,0],risk[:,1],s=12,c='black',label='all observed risk-cell centers');ax.add_patch(plt.Circle((-.4,.1),.8707106781186548,color='gray',alpha=.12));
for trace in r['trace']:
 q=np.asarray(trace['samples']);ax.plot(q[:,0],q[:,1],color='green' if trace['passed'] else 'red',alpha=.8)
ax.scatter(*r['query']['start_xy'],c='blue',s=45,label='actual state at50.6s');ax.set_xlim(-2.1,.15);ax.set_ylim(-.6,1.6);ax.set_aspect('equal');ax.set_xlabel('worldX m');ax.set_ylabel('worldY m');ax.set_title('First .3m arcs pass; all second .3m arcs blocked\nGreen/red: original frozen planner test, gray: nominal margin');ax.legend();fig.tight_layout();fig.savefig(D/'first_prefix_blockage_v1.png',dpi=140)
