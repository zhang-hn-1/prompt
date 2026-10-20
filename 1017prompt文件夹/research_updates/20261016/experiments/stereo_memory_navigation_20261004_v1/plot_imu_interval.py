from pathlib import Path
import json,datetime
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;out=H/'imu_interval_run';z=np.load(H/'imu_interval_measurement.npz');t=z['time_s'];fig,axs=plt.subplots(3,1,figsize=(10,8),sharex=True)
axs[0].plot(t,z['position_error_m']);axs[0].axhline(.1,color='red',linestyle='--');axs[0].set_ylabel('Position error(m)')
axs[1].plot(t,z['attitude_error_deg']);axs[1].axhline(2,color='red',linestyle='--');axs[1].set_ylabel('Attitude error(deg)')
axs[2].plot(t,z['actual_position'][:,0],t,z['predicted_position'][:,0]);axs[2].set_ylabel('Gauge X position(m)');axs[2].set_xlabel('Scored time(s)');fig.tight_layout();fig.savefig(H/'imu_interval_measurement.png');plt.close(fig)
rows=[];strip=[]
for name in ['third_person','stereo_view']:
 cap=cv2.VideoCapture(str(out/(name+'.mp4')));fps=cap.get(cv2.CAP_PROP_FPS);n=0;pictures=[]
 while True:
  ok,frame=cap.read()
  if not ok:break
  if n in [0,160,320]:pictures.append(cv2.resize(frame,(640,360)))
  n+=1
 cap.release();assert n==321 and fps==10. and len(pictures)==3
 strip.append(np.concatenate(pictures,axis=1));rows.append(dict(video=name,decoded_frames=n,fps=fps))
cv2.imwrite(str(H/'imu_interval_actual_video_snapshots.png'),np.concatenate(strip,axis=0));(H/'imu_interval_video_check.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),videos=rows,frames=[0,160,320],actually_decoded=True),indent=2)+'\n');print('FULL_VIDEO_DECODE',rows)
