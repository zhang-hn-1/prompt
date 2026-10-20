"""Plot only sealed arrays/results; no model or source calls."""
from pathlib import Path
import json,hashlib,os,datetime
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
D=Path(__file__).resolve().parent;C=D.parent/'conservative_capture_admission_20261017_v1'
load=lambda p:json.loads(Path(p).read_text())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
cfg=load(D/'protocol.json');fs=load(cfg['feature_seal']);f=[x for x in fs['rows'] if x['sequence']=='E1B' and x['frame']==90][0]
with np.load(f['input_rgb_frame']) as z:rgb=[z[n+'_rgb'] for n in ['front_left','front_right','rear_left','rear_right']]
with np.load(D/'maps/E1B_090.npz') as z:B=z['B_labels'].copy();L=z['Q_labels'].copy();origin=z['origin'].copy();cells=z['cells'].copy();scores=z['model_scores'].copy()
with np.load(C/'maps/E1B_090.npz') as z:Q=z['Q_labels'].copy();assert np.array_equal(origin,z['origin']) and np.array_equal(B,z['B_labels'])
seq=[s for s in cfg['sequences'] if s['name']=='E1B'][0]
with np.load(seq['occupancy']) as z:truthxy=np.argwhere(z['grid']>0)*float(z['resolution_m'])+z['global_origin_xy']
truth=cKDTree(truthxy)
a=[x for x in load(cfg['action_seal'])['rows'] if x['sequence']=='E1B' and x['frame']==90][0]
with np.load(a['path']) as z:phys=z['physical_teacher_class'].copy();assert np.array_equal(cells,z['cells'])
fig,axes=plt.subplots(2,4,figsize=(18,8));fig.suptitle('E1B fixed middle exposure 90, t=45s: learned action L FAILED development gates\nGray=unknown; blue=offline occupied proxy; maps show fixed 10m crop, full400x400 domain scored')
center=origin+20
for ax,img,n in zip(axes[0],rgb,['front_left','front_right','rear_left','rear_right']):ax.imshow(img);ax.set_title(n+' recorded RGB (unchanged)');ax.axis('off')
for ax,labels,title in zip(axes[1,:3],[B,L,Q],['B: all admitted history','L: actual learned admission history','Q: PRIVILEGED teacher diagnostic']):
 ax.set_facecolor('.7');ax.scatter(*truthxy.T,s=3,c='steelblue',label='offline GT bodyband proxy')
 xy=np.argwhere(labels==1)*.1+origin
 if len(xy):
  good=truth.query(xy)[0]<=.15;ax.scatter(*xy[good].T,s=5,c='firebrick',label='risk within .15m');ax.scatter(*xy[~good].T,s=6,c='darkorange',label='false risk >.15m')
 ax.set_title(title);ax.set_xlim(center[0]-5,center[0]+5);ax.set_ylim(center[1]-5,center[1]+5);ax.set_aspect('equal');ax.set_xlabel('world x (m)');ax.set_ylabel('world y (m)');ax.legend(fontsize=6)
ax=axes[1,3];ax.set_facecolor('.7');xy=cells*.1+origin;reject=scores>=.95
for m,c,marker,label in [((phys==1)&~reject,'firebrick','o','positive HOLD'),((phys==-1)&~reject,'khaki','s','physical UNKNOWN HOLD'),((phys==0)&reject,'forestgreen','x','negative WITHHOLD'),((phys==0)&~reject,'darkorange','s','negative MISSED'),((phys!=0)&reject,'purple','x','positive/UNKNOWN WRONG withhold')]:
 if m.any():ax.scatter(*xy[m].T,s=14,c=c,marker=marker,label=label)
ax.set_title('L actual current jointcell decisions');ax.set_xlim(center[0]-5,center[0]+5);ax.set_ylim(center[1]-5,center[1]+5);ax.set_aspect('equal');ax.set_xlabel('world x (m)');ax.set_ylabel('world y (m)');ax.legend(fontsize=6)
fig.tight_layout(rect=(0,0,1,.94),h_pad=3.0);fig.savefig(D/'fixed_middle_learned_admission.png',dpi=140);plt.close(fig)
r=load(D/'result.json');q=load(C/'result.json');fig,axes=plt.subplots(1,3,figsize=(15,4));fig.suptitle('Full frozen-input replay; B vs learned L vs privileged Q. Not online navigation or independent validation.')
for ax,name in zip(axes,['E0','E1B','E1A']):
 rows=r['sequences'][name]['rows'];qr=q['sequences'][name]['rows'];assert len(rows)==len(qr)
 t=[x['time_s'] for x in rows]
 for key,color,label,rr in [('B','darkorange','B baseline',rows),('Q','firebrick','L learned (rejected)',rows),('Q','royalblue','Q privileged diagnostic',qr)]:ax.plot(t,[x[key]['false'] for x in rr],c=color,label=label)
 ax.set_title(name+f': {len(rows)} full score times');ax.set_xlabel('recorded simulated time (s)');ax.set_ylabel('false risk cells >.15m');ax.legend(fontsize=7)
fig.tight_layout(rect=(0,0,1,.92));fig.savefig(D/'complete_learned_vs_privileged_curves.png',dpi=140);plt.close(fig)
receipt=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),pid=os.getpid(),selection='Fixed same E1B180 midpoint index90, no selection from outcome',raw_sha256=sha(f['input_rgb_frame']),learned_map_sha256=sha(D/'maps/E1B_090.npz'),result_sha256=sha(D/'result.json'),GT_plot_only=True,source_calls=0,model_score_calls=0,new_depth_or_error='N/A original source unchanged',per_eye_admission='N/A jointcell action',new_navigation='N/A',figures=['fixed_middle_learned_admission.png','complete_learned_vs_privileged_curves.png'])
(D/'visualization_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n');print('SEALED_LEARNED_PLOTS_ONLY',flush=True)
