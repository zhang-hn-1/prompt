from pathlib import Path
import json,hashlib,sys
import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent;R=D.parent/'wide_pinhole_camera_20261015_v1/native_validation_run';result=json.loads((D/'result.json').read_text());objects=json.loads((D/'scene_cylinders.json').read_text());seal=json.loads((R/'capture_seal.json').read_text());f=584.8888888888889/3
checked=0;maxerr=0.;cases=[]
for rr,row in zip(seal['rows'],result['frames']):
 a=np.load(R/'frames'/rr['frame']); b=np.load(D/(Path(rr['frame']).stem+'_diagnostic.npz'))
 sets=[set(map(tuple,b[f'eye{e}_cells'])) for e in [0,1]];un=sets[0]|sets[1];old=set(map(tuple,b['old_analytic_cells']));assert len(un)==row['union_cells'] and len(sets[1]-sets[0])==row['rear_unique_cells'];assert len(old&un)==row['old_analytic_exact_retained'];assert len(sets[0]&sets[1])==row['overlap_cells']
 assert set(map(tuple,b['union_cells']))==un
 for e in [0,1]:
  T=a[f'eye{e}_camera_usd_world_label']; origin=T[:3,3]
  # Independent scalar quadratic roots and end caps on fixed 5x5 grid.
  for yy in [30,135,240,345,450]:
   for xx in [20,170,320,470,620]:
    ray=T[:3,:3]@np.array([(xx-319.5)/f,-(yy-239.5)/f,-1.]);hits=[]
    for ob in objects:
     c=np.array(ob['center']);v=origin-c;rad=ob['radius'];half=ob['height']/2
     co=[float(ray[0]**2+ray[1]**2),float(2*(v[0]*ray[0]+v[1]*ray[1])),float(v[0]**2+v[1]**2-rad**2)]
     for t in np.roots(co):
      if np.isreal(t):
       t=float(t.real)
       if t>=.01 and abs(v[2]+t*ray[2])<=half:hits.append(t)
     if abs(ray[2])>1e-20:
      for zc in [-half,half]:
       t=(zc-v[2])/ray[2]
       if t>=.01 and (v[0]+t*ray[0])**2+(v[1]+t*ray[1])**2<=rad**2:hits.append(float(t))
    expected=min(hits) if hits else np.inf;actual=b[f'eye{e}_cylinder_expected_z'][yy,xx]
    if np.isfinite(expected):assert np.isfinite(actual);maxerr=max(maxerr,abs(expected-actual));assert abs(expected-actual)<1e-8
    else:assert np.isinf(actual)
    checked+=1
 # Inverse projection all supported world points independently with homogeneous matrices.
 for e in [0,1]:
  yy,xx=np.where(b[f'eye{e}_supported_mask']);z=a[f'eye{e}_depth_label'][yy,xx];op=np.stack([(xx-319.5)*z/f,-(yy-239.5)*z/f,-z,np.ones(len(z))],axis=1);wp=(a[f'eye{e}_camera_usd_world_label']@op.T).T
  cc=np.unique(np.floor((wp[:,:2]-b['origin'])/.1+.5).astype(np.int64),axis=0);assert np.array_equal(cc,b[f'eye{e}_cells'])
 cases.append(rr['frame'])
for p,h in result['input_sha256'].items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h
summary=dict(pass_check=True,complete_frame_set_operations=len(cases),independent_scalar_ray_queries=checked,max_scalar_vector_ray_difference_m=maxerr,all_supported_pixels_homogeneous_projection_exact=True,full_input_identity=True,scope='Independent arithmetic check only; does not repair missing environment geometry or admit a model')
(D/'independent_check.json').write_text(json.dumps(summary,indent=2)+'\n');print(summary)
a=np.load(R/'frames/frame_000.npz');b=np.load(D/'frame_000_diagnostic.npz');row=result['frames'][0]
fig,ax=plt.subplots(2,4,figsize=(17,8),constrained_layout=True)
for e in [0,1]:
 name=['Front','Rear'][e];ar=row['arms'][e]
 ax[e,0].imshow(np.rot90(a[f'eye{e}_rgb']));ax[e,0].set_title(name+' RGB / native 0.5 s')
 for j,key,title,lim,cm in [(1,'gt_z','Renderer depth label (m)',(.5,3),'viridis'),(2,'cylinder_expected_z','Analytic cylinders ONLY (m)',(.5,3),'viridis'),(3,'error','Ray discrepancy (m), NOT model error',(0,.15),'magma')]:
  z=b[f'eye{e}_{key}'].copy();valid=np.isfinite(b[f'eye{e}_cylinder_expected_z'])&(b[f'eye{e}_cylinder_expected_z']>=.5)&(b[f'eye{e}_cylinder_expected_z']<=3)&np.isfinite(b[f'eye{e}_gt_z'])
  if j==3:z[~valid]=np.nan
  else:z[~np.isfinite(z)]=np.nan
  im=ax[e,j].imshow(np.rot90(z),vmin=lim[0],vmax=lim[1],cmap=cm);fig.colorbar(im,ax=ax[e,j],shrink=.7);ax[e,j].set_title(title)
 for j in range(4):ax[e,j].axis('off')
fig.suptitle('Fixed first exposure, E1 / all 8 frames scored. White=unavailable. Color clipped to legend.\nNo monocular prediction, learned error, semantic/free or memory output in this probe.',fontsize=12)
fig.savefig(D/'first_exposure_depth.png',dpi=140);plt.close(fig)
fig,axes=plt.subplots(1,3,figsize=(16,6.8));fig.subplots_adjust(top=.76,bottom=.12,wspace=.28)
origin=b['origin'];root=b['root'];pos=lambda c:c*.1+origin
for j in range(2):
 ax=axes[j]
 for ob in objects:ax.add_patch(plt.Circle(ob['center'][:2],ob['radius'],fill=False,color='0.65',lw=.6))
 ax.scatter(*root[:2],c='k',marker='*',s=90,label='Robot GT')
 ax.set_xlim(root[0]-4,root[0]+4);ax.set_ylim(root[1]-4,root[1]+4);ax.set_aspect('equal');ax.set_xlabel('World X (m)');ax.set_ylabel('World Y (m)');ax.grid(alpha=.15)
for e,col in [(0,'tab:blue'),(1,'tab:orange')]:
 p=pos(b[f'eye{e}_cells']);axes[0].scatter(p[:,0],p[:,1],c=col,s=12,label=['Front','Rear'][e]+f' ({len(p)})')
axes[0].set_title('Actually rendered cylinder-supported BEV\n66 front + 27 rear; overlap 0');axes[0].legend(fontsize=8)
p=pos(b['old_analytic_cells']);axes[1].scatter(p[:,0],p[:,1],facecolors='none',edgecolors='k',s=35,label=f'Old-rig analytic reference ({len(p)})');p=pos(b['union_cells']);axes[1].scatter(p[:,0],p[:,1],c='tab:green',s=10,label=f'Wide rendered union ({len(p)})');axes[1].set_title('Same-pose cylinder-only reference\nNot original RGB / full-scene recall');axes[1].legend(fontsize=8)
p=b['angle_points'];v=b['angle_visible'];axes[2].scatter(p[v,0]-root[0],p[v,1]-root[1],s=3,c='tab:green',label=f'FOV/range supported ({v.sum()}/3600)');axes[2].scatter(p[~v,0]-root[0],p[~v,1]-root[1],s=3,c='0.65',label=f'Blind ({(~v).sum()}/3600)');axes[2].scatter(0,0,marker='*',c='k');axes[2].set_aspect('equal');axes[2].set_title('2 m ring: geometry ONLY\nGrey is unknown, not free');axes[2].legend(fontsize=8);axes[2].set_xlabel('World-relative X (m)');axes[2].set_ylabel('World-relative Y (m)')
fig.suptitle('E1 first exposure 0.5 s (preselected). Grey circles=exported cylinder scene. Blank=unknown.\nNo model predicted BEV, persistent memory, ground false-positive rate or navigation trial.',fontsize=11)
fig.savefig(D/'first_exposure_bev.png',dpi=150);plt.close(fig)
