from pathlib import Path
import json,numpy as np,cv2,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;D=H/'original_E1_direct_memory_navigation_run_v1';O=H/'original_E1_direct_prefix_visuals_v1';O.mkdir(exist_ok=False)
rows=json.loads((D/'states.json').read_text());p=np.asarray([r['position'] for r in rows]);t=np.asarray([r['time_s'] for r in rows]);source=json.loads((D/'source_rows.json').read_text());fig,ax=plt.subplots(1,2,figsize=(10,4));ax[0].plot(p[:,0],p[:,1]);ax[0].scatter(*p[0,:2],label='start');ax[0].scatter(*p[-1,:2],label='end');ax[0].axis('equal');ax[0].set_xlabel('World X (m)');ax[0].set_ylabel('World Y (m)');ax[0].legend();ax[0].set_title('E1: partial54.6s knownpose navigation, NOT complete60s');ax[1].plot([r['time_s'] for r in source],[r['points'] for r in source],marker='.');ax[1].set_xlabel('Exposure time (s)');ax[1].set_ylabel('Selected obstacle point count');fig.tight_layout();fig.savefig(O/'trajectory_and_source.png',dpi=150);plt.close(fig)
cap=cv2.VideoCapture(str(D/'stereo_view.mp4'));selected=[]
for i in [0,60,155,300,425,545]:
 cap.set(cv2.CAP_PROP_POS_FRAMES,i);ok,im=cap.read();assert ok;im=cv2.resize(im,(640,240));cv2.putText(im,f'E1 partial knownpose, t={i/10:.1f}s',(10,20),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,255),1);selected.append(im)
cap.release();cv2.imwrite(str(O/'stereo_contact_sheet.png'),np.vstack([np.hstack(selected[:3]),np.hstack(selected[3:])]))
print('saved',O)
