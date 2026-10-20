from pathlib import Path
import json,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import cv2
H=Path(__file__).resolve().parent;D=H/'moving_surface_A1_run_v1';s=json.loads((D/'states.json').read_text());a=json.loads((D/'postseal_moving_source_assessment_v2_serialization.json').read_text());t=np.array([r['time_s'] for r in s]);p=np.array([r['position'][:2] for r in s]);v=np.linalg.norm(np.array([r['velocity'][:2] for r in s]),axis=1);dist=np.linalg.norm(p-p[0],axis=1)
fig,ax=plt.subplots(1,3,figsize=(15,4),constrained_layout=True);ax[0].plot(p[:,0],p[:,1]);ax[0].plot(*p[0],'go');ax[0].plot(*p[-1],'rx');ax[0].set_aspect('equal');ax[0].set_title('Actual XY: 0.464m, contact0');ax[0].set_xlabel('world X m');ax[0].set_ylabel('world Y m');ax[1].plot(t,dist,label='actual displacement');ax[1].axhline(.35,color='red',ls='--',label='fixed abort budget');ax[1].axhline(.7,color='gray',ls='--',label='center prior');ax[1].axvspan(2,4,alpha=.1,color='green',label='.5 main, aux0');ax[1].legend(fontsize=8);ax[1].set_xlabel('simulation s');ax[1].set_ylabel('m');ax[2].plot([r['time_s'] for r in a['rows']],[r['precision'] for r in a['rows']],label='estimated pose registration');ax[2].axhline(.95,color='red',ls='--');ax[2].set_ylim(0,1.02);ax[2].set_xlabel('simulation s');ax[2].set_ylabel('all emitted points within10cm / total');ax[2].set_title('Subset precision is not obstacle coverage');fig.savefig(D/'moving_source_evidence_overview.png',dpi=150);plt.close(fig)
verified={}
for name in ['third_person','stereo_view']:
 cap=cv2.VideoCapture(str(D/(name+'.mp4')));n=0;fps=cap.get(cv2.CAP_PROP_FPS)
 while True:
  ok,frame=cap.read()
  if not ok:break
  if n in [0,40,80]:cv2.imwrite(str(D/f'{name}_frame_{n:03d}.png'),frame)
  n+=1
 cap.release();assert n==81 and abs(fps-10)<1e-6;verified[name]=dict(decoded_frames=n,fps=fps)
(D/'moving_video_decode_verified_v1.json').write_text(json.dumps(dict(videos=verified,author_decoded_all_frames_not_watched_all=True,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2)+'\n');print(verified)
