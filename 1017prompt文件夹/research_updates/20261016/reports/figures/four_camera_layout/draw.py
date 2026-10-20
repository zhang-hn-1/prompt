from pathlib import Path
import sys,json,numpy as np,cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Polygon
W=Path.cwd();D=W/'reports/figures/four_camera_layout';R=W/'experiments/original_four_camera_20261015_v1'
sys.path.insert(0,str(R));from cad_masks import FourCAD
cfg=json.loads((R/'protocol.json').read_text());a=np.load(R/'native_run/frames/frame_000.npz');b=np.load(R/'frame_000_audit.npz');print(a.files[-8:],flush=True)
c=FourCAD();q=a['joints_current'];T=c.cad.body_transforms(q);parts={k:v@T[k][:3,:3].T+T[k][:3,3] for k,v in c.cad.triangles.items()};print({k:[v.reshape(-1,3).min(0).tolist(),v.reshape(-1,3).max(0).tolist()] for k,v in parts.items()},flush=True)
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':12})
fig,axes=plt.subplots(2,2,figsize=(14,12),gridspec_kw={'height_ratios':[1.1,1]},constrained_layout=True)
colors=['#8194aa','#a69dc9','#c1b398','#96b6aa'];labels=['基体 base_link','关节部件 link1','关节部件 link2','关节部件 link3']
lo,hi=-.65,.65;N=1000
for col,ax in enumerate(axes[0]):
 for (k,v),color,label in zip(parts.items(),colors,labels):
  uv=np.stack([-v[:,:,1],v[:,:,0]],-1) if col==0 else v[:,:,[0,2]]
  pix=np.rint((uv-lo)/(hi-lo)*(N-1)).astype(np.int32);mask=np.zeros((N,N),np.uint8);cv2.fillPoly(mask,list(pix),1)
  rgba=np.zeros((N,N,4));rgba[:,:,:3]=matplotlib.colors.to_rgb(color);rgba[:,:,3]=mask*.5
  ax.imshow(rgba,origin='lower',extent=[lo,hi,lo,hi]);ax.plot([],[],lw=7,color=color,alpha=.7,label=label)
 ax.axhline(0,color='#bcc4cd',ls=':',lw=.8);ax.axvline(0,color='#bcc4cd',ls=':',lw=.8);ax.plot(0,0,'+',color='#344054',ms=10)
 ax.set_xlim(-.61,.61);ax.set_ylim(-.58,.64);ax.set_aspect('equal');ax.grid(alpha=.15);ax.set_xticks(np.arange(-.6,.61,.2));ax.set_yticks(np.arange(-.4,.61,.2))
 ax.set_xlabel('横向：左 ← −Y / +Y → 右（图坐标为 −Y，单位 m）' if col==0 else '后 ← X（m）→ 前')
 if col==0:ax.set_xlabel('机体左侧 ← 横向位置 −Y（m）→ 机体右侧')
 ax.set_ylabel('后 ← X（m）→ 前' if col==0 else 'Z（m，相对 base_link 原点）')
 ax.set_title('俯视：四个位置与光轴\n灰彩轮廓＝实际 CAD 投影，不是球壳假定圆',fontsize=14) if col==0 else ax.set_title('侧视：两侧相机位置重合显示\n镜头高度 +5 cm 是相对机体原点，不是离地高度',fontsize=14)
for j,(name,p) in enumerate(zip(cfg['camera_names'],cfg['positions_m'])):
 p=np.array(p);front=j<2;co='#1269aa' if front else '#cf651d';sgn=1 if front else -1
 xy=np.array([-p[1],p[0]]);axes[0,0].scatter(*xy,c=co,s=95,marker='s',edgecolors='white',zorder=10);axes[0,0].annotate('',xy=xy+[0,.22*sgn],xytext=xy,arrowprops={'arrowstyle':'-|>','color':co,'lw':2.5})
 text=['前左','前右','后左','后右'][j]+f' ({p[0]:+.2f}, {p[1]:+.2f}, +.05)'
 axes[0,0].annotate(text,xy=xy,xytext=(0,12 if front else -24),textcoords='offset points',ha='center',fontsize=10,color=co,fontweight='bold')
 if j%2==0:
  xz=p[[0,2]];axes[0,1].scatter(*xz,c=co,s=110,marker='s',edgecolors='white',zorder=10);axes[0,1].annotate('',xy=xz+[.23*sgn,0],xytext=xz,arrowprops={'arrowstyle':'-|>','color':co,'lw':2.5});axes[0,1].annotate(('前两台' if front else '后两台')+f'\nx={p[0]*100:+.0f}cm, z=+5cm',xy=xz,xytext=(0,-48 if front else 30),textcoords='offset points',ha='center',color=co,fontsize=12)
axes[0,1].legend(loc='lower center',fontsize=9,ncol=2)
for ax,n,title in zip(axes[1],['rear_left','rear_right'],['后左相机实际视图','后右相机实际视图']):
 rgb=np.rot90(a[n+'_rgb']);mask=np.rot90(b[n+'_CAD']);ax.imshow(rgb);rgba=np.zeros((*mask.shape,4));rgba[:,:,:3]=[1,.1,.1];rgba[:,:,3]=mask*.30;ax.imshow(rgba);ax.contour(mask,levels=[.5],colors=['#ff4f4f'],linewidths=.7);ax.set_title(title+f'｜已知机体投影 {mask.mean():.1%}',fontsize=14);ax.set_axis_off()
fig.suptitle('本次仿真的四相机布局与后视遮挡\n前置来自现有代码；后置由你提供的前后对称关系推导，尚非实机测量标定',fontsize=18,fontweight='bold')
fig.supxlabel('E1 固定首帧 0.5 s｜红色半透明区域＝CAD 投影遮罩（不是逐像素遮挡真值）｜仅绘图，未移动相机',fontsize=11)
fig.savefig(D/'layout_and_rear_occlusion.png',dpi=160);plt.close(fig)
(D/'layout_source.json').write_text(json.dumps({'source':str(R/'protocol.json'),'frame':str(R/'native_run/frames/frame_000.npz'),'positions_m':cfg['positions_m'],'yaw_degrees':cfg['yaw_degrees'],'rear_extrinsics':'nominal symmetry, not measured hardware','CAD_parts':list(parts),'plot':'full projected CAD triangles at actual first-frame joints, silhouette is not a depth-tested occluder attribution'},indent=2))
