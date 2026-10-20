"""Independent full-frame target projection/counts and direct failure presentation."""
import json,hashlib,time
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

D=Path(__file__).resolve().parent;r=json.loads((D/'e0_qualification.json').read_text())
root=Path(r['data_root']);labels=D/'e0_recovery_qualification';seal=json.loads((root/'capture_seal.json').read_text())
names=seal['camera_names'];f=seal['initial_local_identity'][0]['optics'][0]/18*640
start=time.monotonic();checked=0;pixels=0;aggregate={m:dict(prediction_cells=0,target_cells=0,correct=0,recovered=0) for m in ['oracle','native']};point_errors={'oracle':[],'native':[]}
category={m:{k:dict(target=0,recovered=0) for k in r['visible_category_grid_instances']} for m in ['oracle','native']}
Y,X=np.indices((480,640))
for row,entry in zip(r['rows'],seal['rows']):
 path=root/'frames'/row['frame'];assert hashlib.sha256(path.read_bytes()).hexdigest()==entry['sha256']
 a=np.load(path);b=np.load(labels/row['frame']);target=set();origin=b['origin']
 for name in names:
  z=a[name+'_depth_label'];T=a[name+'_camera_usd_world_label'];mask=b[name+'_CAD']
  k=np.isfinite(z)&(z>=.5)&(z<=3)&~mask
  x=X[k];y=Y[k];zz=z[k]
  optical=np.stack([(x-319.5)*zz/f,-(y-239.5)*zz/f,-zz,np.ones(len(zz))],axis=0)
  w=(T@optical).T[:,:3]
  valid=(w[:,2]>.05)&(w[:,2]<=1.5)&(np.linalg.norm(w-a['root_position_physics_label'],axis=1)>=.55)
  cells=np.unique(np.floor((w[valid,:2]-origin)/.1+.5).astype(np.int64),axis=0)
  target|=set(map(tuple,cells.tolist()));pixels+=int(valid.sum())
  uv=b[name+'_contact_uv'];ray=np.stack([(uv[:,0]-319.5)/f,-(uv[:,1]-239.5)/f,-np.ones(len(uv)),np.zeros(len(uv))],axis=0)
  actual_z=z[uv[:,1],uv[:,0]];gt=(T@np.stack([ray[0]*actual_z,ray[1]*actual_z,-actual_z,np.ones(len(uv))])).T[:,:3]
  for method,key in [('oracle','_camera_usd_world_label'),('native','_camera_usd_world_estimated')]:
   pose=a[name+key];direction=(pose@ray).T[:,:3]
   depth=np.divide(-pose[2,3],direction[:,2],out=np.full(len(uv),np.inf),where=abs(direction[:,2])>1e-12)
   with np.errstate(invalid='ignore'):world=pose[:3,3]+direction*depth[:,None]
   local=np.array(seal['initial_local_identity'][names.index(name)]['local_matrix'])
   base=pose@np.linalg.inv(local)
   center=a['root_position_physics_label'] if method=='oracle' else base[:3,3]
   valid=(depth>=.5)&(depth<=3)&np.isfinite(world).all(1)&(np.linalg.norm(world-center,axis=1)>=.55)
   cc=np.unique(np.floor((world[valid,:2]-origin)/.1+.5).astype(np.int64),axis=0)
   assert np.array_equal(cc,b[name+'_'+method+'_cells'])
   point_errors[method].extend(np.linalg.norm(world[valid,:2]-gt[valid,:2],axis=1).tolist())
  checked+=1
 assert target==set(map(tuple,b['target_union'].tolist()))
 for method in aggregate:
  union=set.union(*(set(map(tuple,b[n+'_'+method+'_cells'].tolist())) for n in names))
  assert union==set(map(tuple,b[method+'_union'].tolist()))
  p=b[method+'_union']*.1+origin;t=b['target_union']*.1+origin
  d=cKDTree(t).query(p)[0] if len(p) and len(t) else np.full(len(p),np.inf)
  q=cKDTree(p).query(t)[0] if len(p) and len(t) else np.full(len(t),np.inf)
  count=dict(prediction_cells=len(p),target_cells=len(t),correct=int((d<=.15).sum()),recovered=int((q<=.15).sum()))
  for key,value in count.items():assert row[method][key]==value;aggregate[method][key]+=value
  for key in category[method]:
   c=b[key+'_cells']*.1+origin
   rec=int((cKDTree(p).query(c)[0]<=.15).sum()) if len(p) and len(c) else 0
   category[method][key]['target']+=len(c);category[method][key]['recovered']+=rec
for method in aggregate:
 for key,value in aggregate[method].items():assert value==r['aggregate'][method][key]
 assert abs(np.quantile(point_errors[method],.9)-r['aggregate'][method]['contact_point_position_p90_m'])<1e-12
 for key,c in category[method].items():
  for k,value in c.items():assert value==r['visible_category_coverage'][method][key][k]
contacts=json.loads((root/'contacts.json').read_text());assert all(v['available'] for v in contacts)
assert not any(v['contact'] for v in contacts)
result=dict(complete=True,camera_frames_checked=checked,full_GT_bodyband_pixels_reprojected=pixels,
 all_contact_ray_cells_and_pixel_error_quantiles_exact=True,all_unions_category_counts_and_gate_denominators_exact=True,
 contact_samples=len(contacts),actual_obstacle_contact_samples=sum(bool(v['contact']) for v in contacts),
 all_captured_file_SHA_verified=True,wall_s=time.monotonic()-start,
 scope='Independent arithmetic full-sequence verification, not RGB/model performance.')
(D/'independent_check.json').write_text(json.dumps(result,indent=2)+'\n');print(result,flush=True)

font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False})
# Fixed middle exposure; selected by index, not by error or target coverage.
row=r['rows'][len(r['rows'])//2];a=np.load(root/'frames'/row['frame']);b=np.load(labels/row['frame']);tm=row['time_s']
fig,axs=plt.subplots(3,4,figsize=(13,11),constrained_layout=True)
for i,(name,label) in enumerate(zip(names,['前左','前右','后左','后右'])):
 rgb=np.rot90(a[name+'_rgb']);uv=b[name+'_contact_uv']
 axs[0,i].imshow(rgb);axs[0,i].set_title(label+'：实际RGB')
 axs[1,i].imshow(rgb);axs[1,i].scatter(uv[:,1],639-uv[:,0],s=3,c='yellow');axs[1,i].set_title(f'理想接触标签：{len(uv)}像素')
 z=np.rot90(a[name+'_depth_label']);depth=np.where(np.isfinite(z)&(z>0),z,np.nan)
 axs[2,i].imshow(depth,cmap='turbo',vmin=.5,vmax=3);axs[2,i].set_title('渲染深度（m）；白＝不可用')
 for j in range(3):axs[j,i].axis('off')
fig.colorbar(plt.cm.ScalarMappable(norm=plt.Normalize(.5,3),cmap='turbo'),ax=axs[2,:],orientation='horizontal',shrink=.65,label='渲染真值深度（m）；超过范围按色阶截断')
fig.suptitle(f'E0完整54.3秒采集：固定中帧 {tm:.1f} s，四路原始观测',fontsize=18)
fig.supxlabel('黄色边界由真实场景几何＋渲染深度离线生成，要求障碍侧和地面分别可见；不是模型预测。\n没有学习深度、预测误差、地面误报或长期记忆输出，相关指标 N/A。全部110组均评分，画面仅旋转正立。',fontsize=11)
fig.savefig(D/'fixed_middle_RGB_labels_depth.png',dpi=145);plt.close(fig)

fig,axes=plt.subplots(1,3,figsize=(17,6),constrained_layout=True);root_xy=a['root_position_physics_label'][:2]
def points(key):return b[key]*.1+b['origin']-root_xy
t=points('target_union');floating=points('suspended_cells')
for ax in axes[:2]:
 ax.scatter(t[:,0],t[:,1],facecolors='none',edgecolors='#333333',s=45,label=f'完整可见目标：{len(t)}格')
 if len(floating):ax.scatter(floating[:,0],floating[:,1],c='#d54599',marker='x',s=28,label=f'悬空目标：{len(floating)}格')
 ax.scatter(0,0,c='black',marker='*',s=70);ax.set_aspect('equal');ax.grid(alpha=.2)
 ax.set_xlim(-4,4);ax.set_ylim(-4,4);ax.set_xlabel('相对世界 X（m）');ax.set_ylabel('相对世界 Y（m）')
for ax,method,color,title in zip(axes[:2],['oracle','native'],['#00897b','#2678b8'],['理想标签＋渲染相机姿态','同一标签＋因果IMU/编码器姿态']):
 p=points(method+'_union');ax.scatter(p[:,0],p[:,1],c=color,s=18,label=f'接触投影：{len(p)}格')
 ax.set_title(title+f"\n本帧覆盖 {row[method]['recovered']}/{row[method]['target_cells']}");ax.legend(fontsize=8)
ax=axes[2];times=[x['time_s'] for x in r['rows']]
for method,col,label in [('oracle','#00897b','理想姿态'),('native','#2678b8','因果IMU姿态')]:
 ax.plot(times,[100*x[method]['recall'] if x[method]['recall'] is not None else np.nan for x in r['rows']],color=col,label=label)
ax.axhline(80,ls='--',c='#bb3333',label='冻结80%门槛');ax.set_ylim(0,105);ax.set_xlabel('仿真时间（s）');ax.set_ylabel('完整可见障碍格覆盖（%）');ax.grid(alpha=.2);ax.legend(fontsize=9)
ax.set_title('全部110组：理想覆盖41.56%，因果姿态39.87%')
fig.suptitle(f'固定中帧 {tm:.1f} s 的BEV与完整序列结果：接触边界源在拟合前拒绝',fontsize=17)
fig.supxlabel('图中投影均使用理想GT接触像素，不是RGB检测结果。世界XY图不把空白当自由空间。\n完整域11652格实例：理想源覆盖4842；悬空目标2535格实例全部未覆盖；地面相交物体覆盖4842/9117。当前无长期地图或导航增量。',fontsize=11)
fig.savefig(D/'full_sequence_BEV_coverage.png',dpi=145);plt.close(fig)
