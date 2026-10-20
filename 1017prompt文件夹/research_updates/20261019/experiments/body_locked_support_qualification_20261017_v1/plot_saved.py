"""Only sealed advisories/maps and recorded RGB, no selector or source calls."""
from pathlib import Path
import json,hashlib,os,datetime
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
D=Path(__file__).resolve().parent
load=lambda p:json.loads(Path(p).read_text())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
cfg=load(D/'protocol.json');f=[x for x in load(cfg['feature_seal'])['rows'] if x['sequence']=='E1B' and x['frame']==90][0]
names=['front_left','front_right','rear_left','rear_right']
with np.load(f['input_rgb_frame']) as z:rgb=[z[n+'_rgb'] for n in names]
with np.load(f['prediction_path']) as z:uids=[z[n+'_query_pixel'].copy() for n in names]
with np.load(D/'predictions/E1B_frame_090.npz') as z:witness=z['point_witness'].copy();dw=z['distance_world'].copy();db=z['distance_body'].copy();af=int(z['anchor_frame'])
with np.load(D/'maps/E1B_090.npz') as z:B=z['B_labels'].copy();Q=z['Q_labels'].copy();origin=z['origin'].copy();cells=z['cells'].copy();scores=z['advisory_scores'].copy()
with np.load(f['feature_path']) as z:inverse=z['source_cell_row'].copy()
a=[x for x in load(cfg['action_seal'])['rows'] if x['sequence']=='E1B' and x['frame']==90][0]
with np.load(a['path']) as z:phys=z['physical_teacher_class'].copy();assert np.array_equal(cells,z['cells'])
seq=[s for s in cfg['sequences'] if s['name']=='E1B'][0]
with np.load(seq['occupancy']) as z:truthxy=np.argwhere(z['grid']>0)*float(z['resolution_m'])+z['global_origin_xy']
truth=cKDTree(truthxy);center=origin+20
fig,axes=plt.subplots(2,4,figsize=(18,8));fig.suptitle(f'Fixed E1B middle active exposure90, t=45s; anchor={af}. D is an UNTRAINED advisory (rejected).\nRGB unchanged; magenta=point compatibility witness, not a true object. Gray=unknown; blue=offline bodyband proxy.')
offset=0
for ax,img,n,q in zip(axes[0],rgb,names,uids):
 ax.imshow(img);mask=witness[offset:offset+len(q)];offset+=len(q)
 if mask.any():ax.scatter(q[mask]%640,q[mask]//640,s=2,c='magenta',alpha=.65)
 ax.set_title(n+' RGB + point witness');ax.axis('off')
assert offset==len(witness)
for ax,label,title in zip(axes[1,:2],[B,Q],['B: original admitted history','D: actual dual-frame admission history']):
 ax.set_facecolor('.7');ax.scatter(*truthxy.T,s=3,c='steelblue',label='offline occupied proxy');xy=np.argwhere(label==1)*.1+origin
 if len(xy):
  good=truth.query(xy)[0]<=.15;ax.scatter(*xy[good].T,s=5,c='firebrick',label='risk within .15m');ax.scatter(*xy[~good].T,s=6,c='darkorange',label='false risk >.15m')
 ax.set_title(title);ax.set_xlim(center[0]-5,center[0]+5);ax.set_ylim(center[1]-5,center[1]+5);ax.set_aspect('equal');ax.set_xlabel('world x (m)');ax.set_ylabel('world y (m)');ax.legend(fontsize=6)
ax=axes[1,2];ax.set_facecolor('.7');xy=cells*.1+origin;reject=scores==1
for m,c,marker,label in [((phys==1)&~reject,'firebrick','o','positive HOLD'),((phys==-1)&~reject,'khaki','s','physical UNKNOWN HOLD'),((phys==0)&reject,'forestgreen','x','negative WITHHOLD'),((phys==0)&~reject,'darkorange','s','negative MISSED'),((phys!=0)&reject,'purple','x','positive/UNKNOWN wrong withhold')]:
 if m.any():ax.scatter(*xy[m].T,s=14,c=c,marker=marker,label=label)
ax.set_title('D actual jointcell action vs teacher');ax.set_xlim(center[0]-5,center[0]+5);ax.set_ylim(center[1]-5,center[1]+5);ax.set_aspect('equal');ax.set_xlabel('world x (m)');ax.set_ylabel('world y (m)');ax.legend(fontsize=6)
ax=axes[1,3];valid=np.isfinite(dw)&np.isfinite(db);outside=valid&((dw>1)|(db>1))
for value,color,label in [(1,'firebrick','positive proxy'),(-1,'goldenrod','UNKNOWN proxy'),(0,'steelblue','negative proxy')]:
 m=valid&(phys[inverse]==value);ax.scatter(dw[m],db[m],s=2,c=color,alpha=.3,label=label)
ax.axhline(.1,c='black',ls='--',lw=.8);ax.axvline(.2,c='black',ls='--',lw=.8);ax.set_xlim(0,1);ax.set_ylim(0,1);ax.set_xlabel('world nearest-set distance (m)');ax.set_ylabel('body nearest-set distance (m)');ax.set_title(f'Point residuals: finite {valid.sum()}/{len(valid)}\ndisplay [0,1]m, {outside.sum()} beyond axes');ax.legend(fontsize=6)
fig.tight_layout(rect=(0,0,1,.94),h_pad=3.);fig.savefig(D/'fixed_middle_dualframe_advisory.png',dpi=140);plt.close(fig)
r=load(D/'result.json');an=load(D/'analysis/result.json');fig,axes=plt.subplots(2,3,figsize=(15,8));fig.suptitle('Complete frozen-input B/D replay and fixed-condition negative-cell diagnosis; no threshold variants.')
colors=['.65','royalblue','goldenrod','forestgreen'];labels=['some anchor unavailable','world-compatible some point','world-far, not all body-close','fixed rule withholds']
for j,name in enumerate(['E0','E1B','E1A']):
 ax=axes[0,j];rows=r['sequences'][name]['rows'];t=[x['time_s'] for x in rows]
 for key,color,label in [('B','darkorange','B baseline'),('Q','purple','D advisory rejected')]:ax.plot(t,[x[key]['false'] for x in rows],c=color,label=label)
 ax.set_title(name+f': {len(rows)} complete score times');ax.set_xlabel('recorded simulated time (s)');ax.set_ylabel('false risk cells >.15m');ax.legend(fontsize=7)
 ax=axes[1,j];s=an['sequences'][name]['classes']['negative'];parts=[s['missing_anchor_somepoint'],s['some_world_compatible'],s['all_world_far']-s['both_all_withheld'],s['both_all_withheld']];assert sum(parts)==s['total_cells'];left=0
 for v,c,label in zip(parts,colors,labels):
  ax.barh([0],[v],left=left,color=c,label=label);left+=v
 ax.set_title(name+f': full {s["total_cells"]} negative cell-exposures\nanchor-available {s["allpoints_anchor_available"]}, withhold {s["both_all_withheld"]}');ax.set_xlabel('cell-exposure rows (original teacher)');ax.set_yticks([]);ax.legend(fontsize=6,loc='upper center',bbox_to_anchor=(.5,-.2))
fig.tight_layout(rect=(0,0,1,.95),h_pad=3.);fig.savefig(D/'complete_replay_and_failure_layers.png',dpi=140);plt.close(fig)
receipt=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),pid=os.getpid(),selection='Same frozen E1B active180 midpointindex90; no favorableframe selection',raw_sha256=sha(f['input_rgb_frame']),advisory_sha256=sha(D/'predictions/E1B_frame_090.npz'),map_sha256=sha(D/'maps/E1B_090.npz'),result_sha256=sha(D/'result.json'),plot_only_GT=True,source_calls=0,model_calls=0,new_depth_or_RGBsemantics='N/A; original source unchanged',per_eye_output='point compatibility witness only; joint admission and accumulatedmap not per-eye independent measures',new_navigation='N/A')
(D/'visualization_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n');print('SEALED_ARRAY_PLOTS_ONLY',flush=True)
