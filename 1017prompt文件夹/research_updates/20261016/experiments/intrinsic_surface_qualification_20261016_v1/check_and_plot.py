"""Independent homogeneous replay and scope-labelled scientific figures."""
from pathlib import Path
import json, hashlib, time
import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

D=Path(__file__).resolve().parent;W=D.parents[1];H=W/'experiments/contact_boundary_source_20261016_v1'
r=json.loads((D/'result.json').read_text());cfg=json.loads((D/'protocol.json').read_text());root=Path(cfg['data_root'])
seal=json.loads((root/'capture_seal.json').read_text());names=seal['camera_names'];F=seal['initial_local_identity'][0]['optics'][0]/18*640
Y,X=np.indices((480,640));start=time.monotonic();totals={m:np.zeros(4,int) for m in ['oracle','native']};errors=[];pixels=0
arr=lambda v:np.array(sorted(v),int).reshape(-1,2)
to_cells=lambda w,o:set(map(tuple,np.floor((w[:,:2]-o)/.1+.5).astype(int).tolist()))
for row,record in zip(r['rows'],seal['rows']):
 path=root/'frames'/row['frame'];assert hashlib.sha256(path.read_bytes()).hexdigest()==record['sha256']
 a=np.load(path);cache=np.load(H/'e0_recovery_qualification'/row['frame']);saved=np.load(D/'maps'/row['frame']);origin=saved['origin']
 targets=set();sources={};classes={}
 for j,name in enumerate(names):
  z=a[name+'_depth_label'];valid=np.isfinite(z)&(z>=.5)&(z<=3)&~cache[name+'_CAD'];u=X[valid];v=Y[valid];zz=z[valid]
  q=np.stack([(u-319.5)*zz/F,-(v-239.5)*zz/F,-zz,np.ones(len(zz))])
  t=(a[name+'_camera_usd_world_label']@q).T[:,:3];n=(a[name+'_camera_usd_world_estimated']@q).T[:,:3]
  k=(t[:,2]>.05)&(t[:,2]<=1.5)&(np.linalg.norm(t-a['root_position_physics_label'],axis=1)>=.55)
  cl=np.zeros(z.shape,bool);cl[v[k],u[k]]=True;classes[name]=cl
  targets|=to_cells(t[k],origin);errors.extend(np.linalg.norm(n[k,:2]-t[k,:2],axis=1));pixels+=int(k.sum())
  select=k&(u%2==0)&(v%2==0);sources[name]=(q[:,select],j)
 assert np.array_equal(arr(targets),saved['target'])
 for method,key in [('oracle','_camera_usd_world_label'),('native','_camera_usd_world_estimated')]:
  fused=set()
  for name in names:
   q,j=sources[name];pose=a[name+key];world=(pose@q).T[:,:3]
   body=pose@np.linalg.inv(np.asarray(seal['initial_local_identity'][j]['local_matrix']))
   center=a['root_position_physics_label'] if method=='oracle' else body[:3,3]
   world=world[np.linalg.norm(world-center,axis=1)>=.55]
   partner=names[j^1];other=a[partner+key];c=(np.linalg.inv(other)@np.c_[world,np.ones(len(world))].T).T[:,:3];c[:,1:]*=-1
   uv=np.rint(c[:,:2]/np.maximum(c[:,2,None],1e-9)*F+[319.5,239.5]).astype(int);u,v=uv.T
   inside=(c[:,2]>0)&(u>=0)&(u<640)&(v>=0)&(v<480);ix=np.flatnonzero(inside);zz=a[partner+'_depth_label'][v[ix],u[ix]]
   usable=np.isfinite(zz)&(zz>=.5)&(zz<=3)&~cache[partner+'_CAD'][v[ix],u[ix]];ix=ix[usable];zz=zz[usable]
   keep=np.ones(len(world),bool);keep[ix]=abs(zz-c[ix,2])<=.15;fused|=to_cells(world[keep],origin)
  assert np.array_equal(arr(fused),saved[method]), (row['frame'],method)
  p=arr(fused)*.1;t=arr(targets)*.1;pd=cKDTree(t).query(p)[0] if len(p) and len(t) else np.full(len(p),np.inf);td=cKDTree(p).query(t)[0] if len(p) and len(t) else np.full(len(t),np.inf)
  one=np.array([len(p),len(t),int((pd<=.15).sum()),int((td<=.15).sum())]);assert np.array_equal(one,[row[method][k] for k in ['predicted','target','correct','covered']]);totals[method]+=one
for m in totals:assert np.array_equal(totals[m],[r['aggregate'][m][k] for k in ['predicted','target','correct','covered']])
assert np.allclose(np.quantile(errors,[.5,.9,.95]),r['native_pose_target_pixel_xy_p50_p90_p95_m'],atol=1e-12,rtol=0)
check=dict(complete=True,eye_frames=440,full_target_pixels=pixels,target_sets_exact=True,four_view_fused_sets_exact=True,full_counts_and_continuous_quantiles_exact=True,all_capture_SHA_exact=True,wall_s=time.monotonic()-start)
(D/'independent_check.json').write_text(json.dumps(check,indent=2)+'\n');print(check,flush=True)

font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False})
row=r['rows'][len(r['rows'])//2];a=np.load(root/'frames'/row['frame']);cache=np.load(H/'e0_recovery_qualification'/row['frame']);b=np.load(D/'maps'/row['frame']);tm=row['time_s']
fig,axs=plt.subplots(3,4,figsize=(13,11),constrained_layout=True)
for j,(name,label) in enumerate(zip(names,['前左','前右','后左','后右'])):
 rgb=np.rot90(a[name+'_rgb']);axs[0,j].imshow(rgb);axs[0,j].set_title(label+'：实际RGB')
 z=a[name+'_depth_label'];valid=np.isfinite(z)&(z>=.5)&(z<=3)&~cache[name+'_CAD'];zz=z[valid];u=X[valid];v=Y[valid]
 q=np.stack([(u-319.5)*zz/F,-(v-239.5)*zz/F,-zz,np.ones(len(zz))]);true=(a[name+'_camera_usd_world_label']@q).T[:,:3];native=(a[name+'_camera_usd_world_estimated']@q).T[:,:3]
 use=(true[:,2]>.05)&(true[:,2]<=1.5)&(np.linalg.norm(true-a['root_position_physics_label'],axis=1)>=.55)
 error=np.full(z.shape,np.nan);error[v[use],u[use]]=np.linalg.norm(native[use,:2]-true[use,:2],axis=1)
 axs[1,j].imshow(np.rot90(np.where(np.isfinite(z)&(z>0),z,np.nan)),vmin=.5,vmax=3,cmap='turbo');axs[1,j].set_title('渲染真实深度（m）')
 axs[2,j].imshow(np.rot90(error),vmin=0,vmax=.05,cmap='magma');axs[2,j].set_title('仅位姿造成的XY误差（m）')
 for ax in axs[:,j]:ax.axis('off')
fig.colorbar(plt.cm.ScalarMappable(norm=plt.Normalize(.5,3),cmap='turbo'),ax=axs[1,:],orientation='horizontal',shrink=.65,label='真实深度（m）；范围外按色阶截断；白＝无有效渲染深度')
fig.colorbar(plt.cm.ScalarMappable(norm=plt.Normalize(0,.05),cmap='magma'),ax=axs[2,:],orientation='horizontal',shrink=.65,label='连续配准XY误差（m）；白＝原目标域外/CAD/无标签')
fig.suptitle(f'固定中帧 {tm:.1f} s：四路RGB、真实深度、仅位姿的配准误差',fontsize=17)
fig.supxlabel('全部110组440眼帧均评分；此图按中帧索引固定选取，画面仅旋转正立。\n使用理想GT类别与真实深度，不是RGB模型输出；无模型深度误差、实际地面误报或长期地图（N/A）。',fontsize=11)
fig.savefig(D/'fixed_middle_RGB_truth_registration.png',dpi=145);plt.close(fig)

fig,axes=plt.subplots(1,3,figsize=(17,6),constrained_layout=True);xy=a['root_position_physics_label'][:2]
points=lambda name:b[name]*.1+b['origin']-xy
t=points('target');floating=points('suspended')
for ax,m,c,title in zip(axes[:2],['oracle','native'],['#00897b','#2678b8'],['理想类别/深度＋渲染姿态','同一类别/深度＋因果IMU姿态']):
 ax.scatter(t[:,0],t[:,1],facecolors='none',edgecolors='#333333',s=45,label=f'完整目标：{len(t)}格')
 ax.scatter(floating[:,0],floating[:,1],c='#d54599',marker='x',s=45,label=f'悬空目标：{len(floating)}格')
 p=points(m);ax.scatter(p[:,0],p[:,1],c=c,s=13,label=f'表面投影：{len(p)}格');ax.scatter(0,0,c='black',marker='*',s=70)
 ax.set_xlim(-4,4);ax.set_ylim(-4,4);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('相对世界 X（m）');ax.set_ylabel('相对世界 Y（m）');ax.set_title(title+f"\n本帧覆盖 {row[m]['covered']}/{row[m]['target']}");ax.legend(fontsize=9)
ax=axes[2]
for m,c,label in [('oracle','#00897b','渲染姿态'),('native','#2678b8','因果IMU姿态')]:ax.plot([x['time_s'] for x in r['rows']],[x[m]['recall']*100 for x in r['rows']],c=c,label=label)
ax.axhline(80,ls='--',c='#bb3333',label='冻结80%门槛');ax.set_ylim(0,105);ax.grid(alpha=.2);ax.legend();ax.set_xlabel('仿真时间（s）');ax.set_ylabel('完整目标覆盖（%）');ax.set_title('全部110组：99.8455% / 99.8541%')
fig.suptitle('完整四路消费者的必要oracle通过；在线感知源仍未准入',fontsize=17)
fig.supxlabel('原完整分母11652格实例；因果投影覆盖11635，悬空2535/2535；GT类别不在线使用。\n格P90＝0来自0.1m栅格和最近集合评分，连续像素XY误差P90＝0.032901m；空白仍未知，无长期地图/导航结果。',fontsize=11)
fig.savefig(D/'full_sequence_surface_BEV.png',dpi=145);plt.close(fig)
