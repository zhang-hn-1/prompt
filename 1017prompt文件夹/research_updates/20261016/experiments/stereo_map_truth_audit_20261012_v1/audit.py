from pathlib import Path
import sys,json,re,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;W=D.parents[1];V=W/'experiments/viewpoint_confirmation_navigation_20261012_v1';R=V/'candidate_e2_run'
sys.path[:0]=[str(V),str(W/'experiments/stereo_memory_navigation_20261004_v1'),'/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm']
from viewpoint_confirmation import ViewpointConfirmationGrid
scene=Path('/home/an/project/NavDP/assets/scenes/benchmark_e2/e2_corridor.usda');s=scene.read_text();objects=[]
for typ,name,body in re.findall(r'def (Cylinder|Sphere) "([^"]+)"[^\{]*\{([^}]+)\}',s):
 radius=float(re.search(r'double radius = ([\d.]+)',body)[1]);xyz=[float(x) for x in re.search(r'xformOp:translate = \(([^)]+)',body)[1].split(',')];objects.append((typ,name,radius,xyz))
assert len(objects)>10
occ=np.load('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/data/benchmark_v1/occupancy/e2_corridor_bodyband.npz');assert hashlib.sha256(scene.read_bytes()).hexdigest()==str(occ['scene_sha256'])
def distance(xy):
 if not len(xy):return np.empty(0)
 return np.min(np.stack([np.maximum(np.linalg.norm(xy-np.array(o[3][:2]),axis=1)-o[2],0) for o in objects]),axis=0)
def quant(a):
 return {f'p{q}_m':float(np.percentile(a,q)) for q in [50,90,95]} if len(a) else {}
first=np.load(R/'maps/frame_000.npz');g=ViewpointConfirmationGrid(first['origin'],10,static_scene_assumption=True)
views={int(p.stem.split('_')[1]):dict(time_s=float(np.load(p)['time_s']),position_world=np.load(p)['left_camera_usd_world'][:3,3]) for p in (R/'known_poses').glob('*.npz')}
native=json.loads((R/'native_predictions.json').read_text());poses={round(x['time_s'],7):x['position_world'] for x in native if x.get('available')}
errors=[];xyerrors=[];source_errors=[];registration=[];floor=0;validn=0;total=0;seen=set();provenance={};pointrows=[];snapshots={};timeline=[]
for p in sorted((R/'partial_observations').glob('*.npz')):
 i=int(p.stem.split('_')[1]);a=np.load(p);f=np.load(R/'frames'/p.name);q=a['query_pixel_id'].astype(int);x=q%640;y=q//640;z=f['left_depth_m'][y,x];ok=np.isfinite(z)&(z>0);world=a['world'][ok];rays=np.stack([(x[ok]-319.5)/584.8888888888889,(y[ok]-239.5)/584.8888888888889,np.ones(ok.sum())],1);T=f['left_camera_usd_world'];rot=T[:3,:3]@np.diag([1,-1,-1]);gt=T[:3,3]+(rays*z[ok,None])@rot.T
 est=np.load(R/'known_poses'/p.name)['left_camera_usd_world'];rotest=est[:3,:3]@np.diag([1,-1,-1]);local=(world-est[:3,3])@rotest;undertruth=T[:3,3]+local@rot.T
 e=np.linalg.norm(world-gt,axis=1);errors.extend(e);source_errors.extend(np.linalg.norm(undertruth-gt,axis=1));xyerrors.extend(np.linalg.norm(world[:,:2]-gt[:,:2],axis=1));registration.extend(np.linalg.norm(world-undertruth,axis=1));isfloor=np.abs(gt[:,2])<=.03;floor+=int(isfloor.sum());validn+=len(gt);total+=len(q)
 cells=g.cells(world[:,:2]);gtcells=g.cells(gt[:,:2]);obstacle=(gt[:,2]>.05)&(gt[:,2]<=1.5)
 seen.update(map(tuple,gtcells[obstacle]));
 for c,b in zip(cells,isfloor):
  k=tuple(c);n,fl=provenance.get(k,(0,0));provenance[k]=(n+1,fl+int(b))
 pointrows.append(dict(frame=i,accepted=len(q),truth_valid=len(gt),floor_false_points=int(isfloor.sum()),**quant(e)))
 if i in [200,400,600,895]:snapshots[i]=dict(world=world,gt=gt,isfloor=isfloor)
for i in range(900):
 p=R/'maps'/f'frame_{i:03d}.npz';a=np.load(p);t=float(a['time_s']);rs=json.loads((R/'raw_point_memory_snapshots'/(p.stem+'.json')).read_text())['records'];g.consume_records(rs,t,np.array(poses[round(t,7)]),views);assert np.array_equal(g.labels,a['labels']) and np.array_equal(g.codes,a['codes'])
 if i%5==0 or i==899:
  c=np.argwhere(g.labels==1);persistent=np.argwhere(np.isfinite(g._confirmed_clock));older=np.argwhere(np.isfinite(g._confirmed_clock)&(t-g._confirmed_clock>10));row=dict(frame=i,time_s=t,hazard_cells=len(c),persistent_cells=len(persistent),older10s_cells=len(older),hazard_farther_010m=int((distance(g.origin+c*g.resolution)>.1).sum()),persistent_farther_010m=int((distance(g.origin+persistent*g.resolution)>.1).sum()),older_farther_010m=int((distance(g.origin+older*g.resolution)>.1).sum()));timeline.append(row)
 if i in [200,400,600,899]:
  snapshots.setdefault(i,{}).update(cells=np.argwhere(g.labels==1),persistent=np.argwhere(np.isfinite(g._confirmed_clock)),older=np.argwhere(np.isfinite(g._confirmed_clock)&(t-g._confirmed_clock>10)),time=t)
final=np.argwhere(g.labels==1);persistent=np.argwhere(np.isfinite(g._confirmed_clock));seeninside=[c for c in seen if g.inside(np.array([c]))[0]];covered=sum(any(g.labels[tuple(np.array(c)+np.array(off))]==1 for off in [(0,0),(1,0),(-1,0),(0,1),(0,-1)]) for c in seeninside if 0<c[0]<399 and 0<c[1]<399)
result=dict(source_run=str(R),scene_sha256=str(occ['scene_sha256']),objects=len(objects),accepted_point_instances=total,truth_valid_instances=validn,false_floor_instances=floor,false_floor_fraction=floor/validn,accepted_point_3d_error=quant(errors),accepted_point_xy_error=quant(xyerrors),native_registration_component=quant(registration),stereo_component_with_truth_pose=quant(source_errors),final=timeline[-1],observed_truth_obstacle_cells=len(seeninside),observed_truth_cells_covered_with_cross_1cell=covered,persistent_cells_all_valid_sources_floor=sum(provenance.get(tuple(c),(0,0))[0]>0 and provenance[tuple(c)][0]==provenance[tuple(c)][1] for c in persistent),point_rows=pointrows,timeline=timeline,limits=['Accepted points only, not all RGB pixels; repeated instances not independent.','Reference uses exact analytic XY footprint of scene cylinders and spheres; 0.10m tolerance is explicit evaluation tolerance, not online permission.','Sparse surface map does not certify obstacle volumes or free space.','Observed-surface coverage uses only accepted stereo query pixels whose renderer truth is positive-height; not whole visible-image recall.','All900 saved maps exactly reproduced; truth only evaluator.'])
(D/'metrics.json').write_text(json.dumps(result,indent=2)+'\n')
track=np.array([x['position'][:2] for x in json.loads((R/'states.json').read_text())]);fig,axs=plt.subplots(2,2,figsize=(13,10))
for ax,i in zip(axs.flat,[200,400,600,899]):
 sn=snapshots[i]
 for typ,name,rad,xyz in objects:ax.add_patch(plt.Circle(xyz[:2],rad,color='0.78',ec='0.35',lw=.5))
 c=sn['cells'];xy=g.origin+c*g.resolution;bad=distance(xy)>.1
 ax.scatter(xy[~bad,0],xy[~bad,1],s=12,c='#208b48',label='Risk near true obstacle');ax.scatter(xy[bad,0],xy[bad,1],s=16,c='#d73027',label='Risk >0.10m outside obstacle')
 old=g.origin+sn['older']*g.resolution;ax.scatter(old[:,0],old[:,1],s=45,facecolors='none',edgecolors='#7028a0',label='Persistent age >10s')
 ax.plot(track[:i+1,0],track[:i+1,1],color='#2166ac',lw=1);ax.plot(*track[i], 'o',color='#2166ac');ax.plot(4.45,-.05,'*',color='black',ms=12);ax.set(xlim=(-5.5,5.5),ylim=(-5,4),aspect='equal',title=f'{sn["time"]:.1f}s: gray=truth; blank=unknown',xlabel='World X (m)',ylabel='World Y (m)')
axs[0,0].legend(fontsize=8,loc='upper right');fig.suptitle('Stereo risk map and long-term memory vs scene truth — E2',fontsize=15);fig.tight_layout();fig.savefig(D/'map_comparison.png',dpi=160);plt.close(fig)
# Same-pixel depth visualization; invalid predictions remain blank.
i=400;f=np.load(R/'frames'/f'frame_{i:03d}.npz');pred=np.load(R/'source_predictions'/f'frame_{i:03d}.npz');depth=pred['depth_m'];conf=pred['confidence'];gt=f['left_depth_m'];mask=(conf>0)&np.isfinite(depth)&(depth>=.5)&(depth<=3)&np.isfinite(gt)&(gt>0);err=np.where(mask,np.abs(depth-gt),np.nan)
fig,axs=plt.subplots(1,4,figsize=(17,4));axs[0].imshow(f['left_rgb']);axs[0].set_title('Left RGB (40s)');axs[1].imshow(np.where(mask,depth,np.nan),vmin=.5,vmax=3,cmap='viridis');axs[1].set_title('Confident stereo depth (m)');axs[2].imshow(np.where(np.isfinite(gt)&(gt>0)&(gt<=3),gt,np.nan),vmin=.5,vmax=3,cmap='viridis');axs[2].set_title('Renderer true depth (m)');im=axs[3].imshow(err,vmin=0,vmax=.3,cmap='magma');axs[3].set_title('Same-pixel absolute error (m)');fig.colorbar(im,ax=axs[3],shrink=.65)
for ax in axs:ax.axis('off')
fig.tight_layout();fig.savefig(D/'depth_comparison.png',dpi=150);plt.close(fig)
print(json.dumps({k:v for k,v in result.items() if k not in ['timeline','point_rows']},indent=2))
