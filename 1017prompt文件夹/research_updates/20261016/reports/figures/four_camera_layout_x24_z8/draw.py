from pathlib import Path
import sys,json,numpy as np,cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Polygon
W=Path.cwd();D=W/'reports/figures/four_camera_layout_x24_z8';R=W/'experiments/original_four_camera_20261015_v1'
sys.path.insert(0,str(R));from cad_masks import FourCAD
cfg=json.loads((R/'protocol.json').read_text());a=np.load(R/'native_run/frames/frame_000.npz');b=np.load(R/'frame_000_audit.npz');print(a.files[-8:],flush=True)
cfg['positions_m']=[[.24,.3,.08],[.24,-.3,.08],[-.24,.3,.08],[-.24,-.3,.08]]
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
 ax.set_title('俯视：四个位置与光轴\n灰彩轮廓＝实际 CAD 投影，不是球壳假定圆',fontsize=14) if col==0 else ax.set_title('侧视：两侧相机位置重合显示\n镜头高度 +8 cm 是相对机体原点，不是离地高度',fontsize=14)
for j,(name,p) in enumerate(zip(cfg['camera_names'],cfg['positions_m'])):
 p=np.array(p);front=j<2;co='#1269aa' if front else '#cf651d';sgn=1 if front else -1
 xy=np.array([-p[1],p[0]]);axes[0,0].scatter(*xy,c=co,s=95,marker='s',edgecolors='white',zorder=10);axes[0,0].annotate('',xy=xy+[0,.22*sgn],xytext=xy,arrowprops={'arrowstyle':'-|>','color':co,'lw':2.5})
 text=['前左','前右','后左','后右'][j]+f' ({p[0]:+.2f}, {p[1]:+.2f}, +.08)'
 axes[0,0].annotate(text,xy=xy,xytext=(0,12 if front else -24),textcoords='offset points',ha='center',fontsize=10,color=co,fontweight='bold')
 if j%2==0:
  xz=p[[0,2]];axes[0,1].scatter(*xz,c=co,s=110,marker='s',edgecolors='white',zorder=10);axes[0,1].annotate('',xy=xz+[.23*sgn,0],xytext=xz,arrowprops={'arrowstyle':'-|>','color':co,'lw':2.5});axes[0,1].annotate(('前两台' if front else '后两台')+f'\nx={p[0]*100:+.0f}cm, z=+8cm',xy=xz,xytext=(0,-48 if front else 30),textcoords='offset points',ha='center',color=co,fontsize=12)
axes[0,1].legend(loc='lower center',fontsize=9,ncol=2)

from four_camera import nominal_transforms
from cad_masks import mask_triangles
tri=np.concatenate(list(parts.values()));masks={};statistics={}
for name,Tcam in zip(cfg['camera_names'],nominal_transforms(.24,.3,.08)):
 optical=(tri-Tcam[:3,3])@(Tcam[:3,:3]@np.diag([1,-1,-1]))
 m=mask_triangles(optical,640,480,cfg['focal_px'],319.5,239.5).astype(bool);masks[name]=m
 old=b[name+'_CAD'];statistics[name]={'new_pixels':int(m.sum()),'old_pixels':int(old.sum()),'pixels':307200,'new_fraction':float(m.mean()),'old_fraction':float(old.mean())}
for ax,names,tit in zip(axes[1],[cfg['camera_names'][:2],cfg['camera_names'][2:]],['前两台：左 / 右','后两台：左 / 右']):
 ims=[]
 for name in names:
  old=np.rot90(b[name+'_CAD']);new=np.rot90(masks[name]);im=np.ones((*new.shape,3));im[old]=[.72,.76,.81];im[new]=[.88,.28,.22];ims.append(im)
 gap=np.ones((ims[0].shape[0],35,3));ax.imshow(np.concatenate([ims[0],gap,ims[1]],axis=1));ax.set_axis_off()
 ss=[statistics[n] for n in names];ax.set_title(tit+'（CAD 投影，非新 RGB）\n'+'   ｜   '.join(f"{s['old_fraction']:.1%} → {s['new_fraction']:.1%}" for s in ss),fontsize=14)
for j,(x,y) in enumerate([(.2,.3),(.2,-.3),(-.2,.3),(-.2,-.3)]):
 axes[0,0].scatter(-y,x,s=80,facecolors='none',edgecolors='#313b49',linestyle='--',zorder=11)
for x in [-.2,.2]:axes[0,1].scatter(x,.05,s=80,facecolors='none',edgecolors='#313b49',linestyle='--',zorder=11)
fig.suptitle('更新预览：x = ±24 cm，y = ±30 cm，z = +8 cm\n实心方块＝新位置；空心圆＝旧位置；朝向、光学与机体姿态保持一致',fontsize=18,fontweight='bold')
fig.supxlabel('红＝新位置的机体投影；灰＝仅旧位置有投影；白＝无已知机体投影（不代表场景无遮挡）\n沿用 E1 0.5 s 关节姿态，仅几何预览，未生成新位置的 RGB 或导航结果',fontsize=11)
fig.savefig(D/'layout_x24_z8.png',dpi=160);plt.close(fig)
(D/'comparison.json').write_text(json.dumps({'positions_m':cfg['positions_m'],'yaw_degrees':cfg['yaw_degrees'],'same_pose_frame':str(R/'native_run/frames/frame_000.npz'),'statistics':statistics,'scope':'single saved pose CAD-only preview; no new RGB, no physical/model/navigation run; not real hardware calibration'},indent=2))
print(json.dumps(statistics),flush=True)
