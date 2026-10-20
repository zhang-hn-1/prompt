from pathlib import Path
import json,datetime
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;out=H/'native_imu_run';z=np.load(H/'native_imu_measurement.npz');t=z['time_s'];fig,axs=plt.subplots(3,1,figsize=(10,8),sharex=True)
axs[0].plot(t,z['position_error_m']);axs[0].axhline(.1,color='red',linestyle='--');axs[0].set_ylabel('Position error(m)')
axs[1].plot(t,z['attitude_error_deg']);axs[1].axhline(2,color='red',linestyle='--');axs[1].set_ylabel('Attitude error(deg)')
axs[2].plot(t,z['acceleration_norm'],label='native specific force norm');axs[2].plot(t,z['acceleration_direction_label_angle_deg'],label='acc vs GT gravity(deg; label diagnostic)');axs[2].legend();axs[2].set_xlabel('Scored time(s)');fig.tight_layout();fig.savefig(H/'native_imu_measurement.png');plt.close(fig)
rows=[];strip=[]
for name in ['third_person','stereo_view']:
 cap=cv2.VideoCapture(str(out/(name+'.mp4')));fps=cap.get(cv2.CAP_PROP_FPS);n=0;pictures=[]
 while True:
  ok,frame=cap.read()
  if not ok:break
  if n in [0,153,307]:pictures.append(cv2.resize(frame,(640,360)))
  n+=1
 cap.release();assert n==308 and fps==10. and len(pictures)==3
 strip.append(np.concatenate(pictures,axis=1));rows.append(dict(video=name,decoded_frames=n,fps=fps))
cv2.imwrite(str(H/'native_imu_actual_video_snapshots.png'),np.concatenate(strip,axis=0));(H/'native_imu_video_check.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),videos=rows,frames=[0,153,307],actually_decoded=True),indent=2)+'\n');print('FULL_VIDEO_DECODE',rows)
