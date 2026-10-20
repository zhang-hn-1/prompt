from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
W=Path.cwd(); R=W/'experiments/original_four_camera_20261015_v1'; D=W/'experiments/camera_occlusion_identity_20261015_v1'
f=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']=f.get_name()
a=np.load(R/'frame_000_audit.npz')
fig,axs=plt.subplots(2,4,figsize=(12,8),constrained_layout=True)
for j,(n,t) in enumerate(zip(['front_left','front_right','rear_left','rear_right'],['前左','前右','后左','后右'])):
 m=np.rot90(a[n+'_CAD']).astype(bool)
 for row in range(2):
  rgb=np.ones((*m.shape,3))*.97
  if row==0:rgb[m]=[.85,.23,.18]
  axs[row,j].imshow(rgb);axs[row,j].set_xticks([]);axs[row,j].set_yticks([])
  axs[row,j].set_title(t+('：%.2f%%'%(m.mean()*100) if row==0 else '：0%'))
axs[0,0].set_ylabel('旧采集位置\n|x|=20 cm，z=5 cm',fontsize=13)
axs[1,0].set_ylabel('用户指定预览\n|x|=24 cm，z=8 cm',fontsize=13)
fig.suptitle('机体遮挡投影分开展示（不是新相机 RGB）',fontsize=18)
fig.supxlabel('红色＝已知机体 CAD 投影；白色＝无该投影，不代表场景自由\n上排展示固定首帧；下排全部 8 姿态 × 4 相机均为 0 / 307200 像素。',fontsize=12)
fig.savefig(D/'separate_masks.png',dpi=140)
