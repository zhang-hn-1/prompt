from pathlib import Path
import numpy as np,json,hashlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
D=Path(__file__).resolve().parent;R=D/'recovery_run'
s=json.loads((R/'capture_seal.json').read_text());assert len(s['rows'])==8
for r in s['rows']:assert hashlib.sha256((R/'frames'/r['frame']).read_bytes()).hexdigest()==r['sha256']
f=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']=f.get_name()
fig,axs=plt.subplots(2,4,figsize=(12,8),constrained_layout=True)
for i,k in enumerate([0,7]):
 a=np.load(R/'frames'/f'frame_{k:03d}.npz')
 for j,(name,label) in enumerate(zip(s['camera_names'],['前左','前右','后左','后右'])):
  axs[i,j].imshow(np.rot90(a[name+'_rgb']));axs[i,j].set_title(label+'  %.1f s'%float(a['source_time_s']));axs[i,j].axis('off')
fig.suptitle('仅位置与视场角更新：实际仿真 RGB：x=±18.197 cm，y=±32.506 cm，z=11.503 cm',fontsize=17)
fig.supxlabel('固定展示第1与第8组曝光；仅旋转图像便于观看，未去除机体、未裁剪或填补遮挡。\n四路同步采集，z相对机体原点；这是仿真预览，不是硬件拍摄或导航结果。',fontsize=11)
fig.savefig(D/'actual_rgb_first_last.png',dpi=150)
(D/'preview_check.json').write_text(json.dumps(dict(frames=8,eyes=32,sha_verified=True,max_pair_matrix_error=max(r['pair_error'] for r in s['rows']),all_imu=all(r['imu']['qualified'] for r in s['rows'])),indent=2))
