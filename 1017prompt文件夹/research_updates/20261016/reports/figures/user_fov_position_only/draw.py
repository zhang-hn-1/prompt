from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Arc
from matplotlib.font_manager import FontProperties
D=Path(__file__).resolve().parent;W=D.parents[2]
s=json.loads((W/'experiments/user_fov_position_only_20261015_v1/recovery_run/capture_seal.json').read_text())
f,w,h,*_=s['initial_local_identity'][0]['optics'];ah=2*np.arctan(h/(2*f));av=2*np.arctan(w/(2*f));span=4*np.tan(ah/2)
font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':11})
fig,axs=plt.subplots(1,2,figsize=(13,8),constrained_layout=True)
ax=axs[0]
for x,y,label in [( .18197,.32506,'前左'),(.18197,-.32506,'前右'),(-.18197,.32506,'后左'),(-.18197,-.32506,'后右')]:
 sg=1 if x>0 else -1;co='#1976be' if x>0 else '#d56a21';p=np.array([-y,x]);end=x+2*sg
 ax.add_patch(Polygon([p,[-y-span/2,end],[-y+span/2,end]],closed=True,facecolor=co,alpha=.14,edgecolor=co,lw=1.4))
 ax.plot([-y,-y],[x,end],color=co,ls='--',lw=.8)
 ax.scatter(*p,s=60,c=co,marker='s',zorder=5);ax.text(-y,x+(.12 if sg>0 else -.19),label,ha='center',color=co,fontweight='bold')
ax.plot(0,0,'k+',ms=10);ax.text(0,.02,'机体原点',ha='center',va='bottom',fontsize=9)
# One camera's width only, not pair union.
ytop=2.18197
ax.annotate('',xy=(-.32506-span/2,ytop),xytext=(-.32506+span/2,ytop),arrowprops=dict(arrowstyle='<->',color='#12517e',lw=1.8))
ax.text(-.32506,ytop+.10,f'单台在前方2 m处：宽约 {span:.2f} m',ha='center',color='#12517e')
ax.text(0,1.25,'每台左右视角\n78.3°',ha='center',fontsize=14,color='#12517e')
ax.text(1.8,0,'侧向\n视野空缺',ha='center',color='#666666');ax.text(-1.8,0,'侧向\n视野空缺',ha='center',color='#666666')
ax.set_title('俯视：前方 ↑，后方 ↓\n蓝＝前两台；橙＝后两台；深色部分为重叠',fontsize=14)
ax.set_xlabel('机体左侧 ← 横向位置 −Y（m）→ 机体右侧');ax.set_ylabel('前后位置 X（m）');ax.set_xlim(-2.2,2.2);ax.set_ylim(-2.55,2.65);ax.set_aspect('equal');ax.grid(alpha=.18)
ax=axs[1];p=np.array([.18197,.11503]);depth=2;half=depth*np.tan(av/2)
ax.add_patch(Polygon([p,[p[0]+depth,p[1]+half],[p[0]+depth,p[1]-half]],closed=True,facecolor='#1976be',alpha=.16,edgecolor='#1976be',lw=2))
ax.plot([p[0],p[0]+depth],[p[1],p[1]],ls='--',color='#1976be');ax.scatter(*p,marker='s',s=90,c='#1976be');ax.plot(0,0,'k+',ms=10)
ax.text(.38,.42,'前相机',ha='center',fontsize=11)
ax.text(1.25,.38,'上下视角 94.7°',ha='center',fontsize=16,color='#12517e')
ax.annotate('',xy=(2.18197,.11503+half),xytext=(2.18197,.11503-half),arrowprops=dict(arrowstyle='<->',color='#12517e',lw=1.8));ax.text(2.32,.11503,f'2 m处\n高约{2*half:.2f} m',va='center',fontsize=12)
ax.annotate('',xy=(.18197,-2.15),xytext=(2.18197,-2.15),arrowprops=dict(arrowstyle='<->',color='#333333'));ax.text(1.18,-2.32,'沿光轴距离 2 m',ha='center')
ax.set_xlim(-.15,3.05);ax.set_ylim(-2.4,2.6);ax.set_aspect('equal');ax.set_xlabel('前后位置 X（m）→ 前');ax.set_ylabel('高度 Z（m，相对机体原点）');ax.grid(alpha=.18)
ax.set_title('侧视：以前相机为例\n后相机朝反方向，视角相同',fontsize=14)
fig.suptitle('当前四相机预览的视场示意',fontsize=21,fontweight='bold')
fig.supxlabel('位置：x=±18.197 cm、y=±32.506 cm、z=11.503 cm；按正立画面方向标注。\n扇形只表示针孔几何视场，绘至光轴前方2 m；未扣场景遮挡，不代表自由空间，也不是360°环视。',fontsize=11)
fig.savefig(D/'fov_diagram.png',dpi=160)
