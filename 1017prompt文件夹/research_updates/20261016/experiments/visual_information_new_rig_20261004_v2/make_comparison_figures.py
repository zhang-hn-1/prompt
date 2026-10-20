"""Static scientific figures from completed old and current measurements."""
from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
H=Path(__file__).resolve().parent;O=H.parent/'visual_information_diagnostic_20261003_v1';F=H/'figures';F.mkdir(exist_ok=True)
C=json.loads((H/'old_new_comparison.json').read_text())
plt.rcParams['font.family']=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name();plt.rcParams['axes.unicode_minus']=False
seqs=['e0','e1','e2','hard_0_e1_0003','hard_0_e1_0004','direct_e1'];names=['E0','E1','E2','H3','H4','Direct E1']
fig,axes=plt.subplots(2,2,figsize=(12,8))
for row,model in enumerate(('da3_metric','unidepth_v2')):
    for col,metric,title in [(0,'p90_m','障碍点三维误差 p90 / m'),(1,'within_0p3m','误差 ≤0.30 m 的比例 / %')]:
        ax=axes[row,col];x=np.arange(6)
        for i,version in enumerate(('old','new')):
            y=[C['depth'][model]['sequences'][seq]['obstacle'][version][metric] for seq in seqs]
            y=[np.nan if v is None else v*(100 if col==1 else 1) for v in y]
            ax.bar(x+(i-.5)*.36,y,.36,label='旧安装 X10/Z10 cm' if version=='old' else '新安装 X20/Z5 cm')
        ax.axhline(.3 if col==0 else 90,c='red',ls='--');ax.set_xticks(x,names);ax.set_title(model);ax.set_ylabel(title);ax.grid(axis='y',alpha=.2);ax.legend(fontsize=8)
        if col==1:ax.set_ylim(0,103)
fig.suptitle('同一批开发轨迹／同一控制状态：改变安装位置后的实测深度评测\n可见像素集合随位置与正确遮挡掩膜改变，分母见报告',fontsize=13);fig.tight_layout(rect=(0,0,1,.93));fig.savefig(F/'old_new_depth_comparison.png',dpi=150);plt.close(fig)
# Exact same development pose, raw pictures separately rendered at each installation.
old_rows=json.loads((O/'offline_manifest.json').read_text())['samples'];new_rows=json.loads((H/'offline_manifest.json').read_text())['samples']
old_row=next(r for r in old_rows if r['sequence']=='hard_0_e1_0004');new_row=next(r for r in new_rows if r['sequence']=='hard_0_e1_0004')
fig,axes=plt.subplots(2,2,figsize=(8,11))
for row,(name,entry) in enumerate([('旧安装 X10/Z10 cm',old_row),('新安装 X20/Z5 cm',new_row)]):
    frames=np.load(entry['rgb_file'])
    for col,eye in enumerate(('left','right')):
        axes[row,col].imshow(np.rot90(frames[eye+'_rgb']));axes[row,col].set_title(name+' '+('左眼' if eye=='left' else '右眼'));axes[row,col].axis('off')
fig.suptitle('0004 首帧：物理状态相同，图像重新渲染；仅软件转正显示',fontsize=13);fig.tight_layout();fig.savefig(F/'old_new_camera_views_h4.png',dpi=150);plt.close(fig)
