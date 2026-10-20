"""Recompute every oracle ray and map aggregate using homogeneous coordinates."""
import json,hashlib,time
import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from qualify import D,INPUT,NAMES

start=time.monotonic();r=json.loads((D/'contact_oracle_result.json').read_text())
seal=json.loads((INPUT/'recovery_run/capture_seal.json').read_text())
f=seal['initial_local_identity'][0]['optics'][0]/18*640
K=np.array([[f,0,319.5],[0,f,239.5],[0,0,1.]])
aggregate=dict(prediction_cells=0,target_cells=0,prediction_correct=0,target_recovered=0)
maxerr=0.;pixels=0;all_dist=[];pixel_errors=[]
for row in r['rows']:
 a=np.load(INPUT/'recovery_run/frames'/row['frame']);s=np.load(D/(row['frame'].replace('.npz','')+'_contact_oracle.npz'))
 union=set()
 for name in NAMES:
  uv=s[name+'_contact_uv'];op=(np.linalg.inv(K)@np.c_[uv,np.ones(len(uv))].T).T
  T=a[name+'_camera_usd_world_label'];rays=np.c_[op[:,0],-op[:,1],-op[:,2],np.zeros(len(op))]
  directions=(T@rays.T).T[:,:3];depth=-T[2,3]/directions[:,2]
  predicted=T[:3,3]+directions*depth[:,None]
  error=float(np.max(abs(predicted[:,:2]-s[name+'_predicted_xy']))) if len(uv) else 0.
  maxerr=max(maxerr,error);assert error<1e-10
  cells=np.unique(np.floor((predicted[:,:2]-s['origin'])/.1+.5).astype(np.int64),axis=0)
  assert np.array_equal(cells,s[name+'_cells']);union|=set(map(tuple,cells.tolist()));pixels+=len(uv)
  z=a[name+'_depth_label'][uv[:,1],uv[:,0]]
  actual=(T@np.c_[op[:,0]*z,-op[:,1]*z,-z,np.ones(len(z))].T).T[:,:3]
  pixel_errors.extend(np.linalg.norm(actual[:,:2]-predicted[:,:2],axis=1).tolist())
 assert union==set(map(tuple,s['oracle_union'].tolist()))
 p=s['oracle_union']*.1+s['origin'];t=s['target_union']*.1+s['origin']
 d=cKDTree(t).query(p)[0];rec=cKDTree(p).query(t)[0];all_dist.extend(d.tolist())
 counts=dict(prediction_cells=len(p),target_cells=len(t),prediction_correct=int((d<=.15).sum()),target_recovered=int((rec<=.15).sum()))
 for k,v in counts.items():assert v==row[k];aggregate[k]+=v
assert aggregate==r['aggregate']
assert abs(np.quantile(all_dist,.9)-r['oracle_cell_position_p90_m'])<1e-12
assert abs(np.quantile(pixel_errors,.9)-r['contact_pixel_position_p90_m'])<1e-12
summary=dict(complete=True,frames=len(r['rows']),camera_frames=32,contact_pixels_checked=pixels,
 maximum_homogeneous_prediction_difference_m=maxerr,all_maps_and_aggregate_exact=True,
 cell_and_pixel_error_quantiles_exact=True,wall_s=time.monotonic()-start,
 scope='Arithmetic and identity verification of the GT-only oracle, not independent RGB source performance')
(D/'contact_independent_check.json').write_text(json.dumps(summary,indent=2)+'\n');print(summary)
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False})
a=np.load(INPUT/'recovery_run/frames/frame_000.npz');s=np.load(D/'frame_000_contact_oracle.npz');b=np.load(D/'frame_000_audit.npz')
fig=plt.figure(figsize=(13,10),constrained_layout=True);gs=fig.add_gridspec(2,4,height_ratios=[1.1,1])
for i,(name,label) in enumerate(zip(NAMES,['前左','前右','后左','后右'])):
 ax=fig.add_subplot(gs[0,i]);ax.imshow(np.rot90(a[name+'_rgb']))
 uv=s[name+'_contact_uv'];ax.scatter(uv[:,1],639-uv[:,0],c='#ffed00',s=2)
 ax.set_title(label+f'：{len(uv)}个接触像素');ax.axis('off')
ax=fig.add_subplot(gs[1,:2]);root=b['root'][:2];p=s['oracle_union']*.1+s['origin']-root;t=s['target_union']*.1+s['origin']-root
ax.scatter(-t[:,1],t[:,0],s=65,facecolors='none',edgecolors='#222222',label=f'完整可见GT目标：{len(t)}格')
ax.scatter(-p[:,1],p[:,0],s=15,c='#008b85',label=f'理想接触标签＋平面投影：{len(p)}格')
ax.scatter(0,0,c='k',marker='^',s=45);ax.set_aspect('equal');ax.set_xlim(-3.4,3.4);ax.set_ylim(-2,3.6);ax.grid(alpha=.2)
ax.set_xlabel('机体左 ← 横向 −Y（m）→ 右');ax.set_ylabel('前后 X（m）');ax.legend(fontsize=9)
ax.set_title('首帧BEV：同一完整目标域；允许0.15 m邻近')
ax=fig.add_subplot(gs[1,2:]);times=[x['time_s'] for x in r['rows']]
ax.plot(times,[100*x['oracle_recall'] for x in r['rows']],'o-',c='#078787',label='理想标签几何覆盖')
ax.axhline(80,color='#aa3333',ls='--',label='原80%覆盖门槛');ax.set_ylim(0,105);ax.set_xlabel('仿真时间（s）');ax.set_ylabel('完整可见目标覆盖（%）');ax.grid(alpha=.2);ax.legend(fontsize=9)
ax.set_title(f"全部8组：{r['aggregate']['target_recovered']}/{r['aggregate']['target_cells']}格实例被覆盖（{r['oracle_recall']*100:.2f}%）")
fig.suptitle('新目标的必要几何检验：仅在可见且有地面支持的真实接触边界定位',fontsize=18)
fig.supxlabel(f"黄色点由场景几何和渲染深度离线生成，并要求边界上方有障碍、下方有地面；不是RGB模型预测。首帧固定选择。\n{r['aggregate']['prediction_correct']}/{r['aggregate']['prediction_cells']}投影格实例距目标≤0.15 m，格P90={r['oracle_cell_position_p90_m']:.2f} m；当前只有落地圆柱，悬空例为0；尚无完整序列或在线源资格。",fontsize=11)
fig.savefig(D/'contact_oracle.png',dpi=150)
