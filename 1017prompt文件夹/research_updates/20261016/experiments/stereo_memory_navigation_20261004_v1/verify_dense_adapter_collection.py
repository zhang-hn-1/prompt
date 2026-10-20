from pathlib import Path
import json,hashlib,datetime
import numpy as np
import cv2
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
seal=json.loads((H/'dense_adapter_heldout_seal.json').read_text());n=0
for field in ('artifact_sha256','sanitized_rgb_sha256'):
 for f,d in seal[field].items():assert sha(f)==d,f;n+=1
out=H/'dense_adapter_heldout_run';ids=json.loads((out/'identity.json').read_text());assert ids['camera_rig']['rig_id']=='ball_stereo_front020_up005_v2'
clocks=json.loads((out/'camera_clocks.json').read_text());assert [r['frame'] for r in clocks]==list(range(321));assert all(abs(r['episode_time_s']-i*.1)<1e-5 for i,r in enumerate(clocks))
states=json.loads((out/'states.json').read_text());assert len(states)==321
videos={};images=[]
for name in ('stereo_view','third_person'):
 cap=cv2.VideoCapture(str(out/(name+'.mp4')));fps=cap.get(cv2.CAP_PROP_FPS);i=0;snaps=[]
 while True:
  ok,bgr=cap.read()
  if not ok:break
  if i in (0,160,320):snaps.append(cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB))
  i+=1
 cap.release();assert i==321 and abs(fps-10)<1e-8
 videos[name]=dict(decoded_frames=i,fps=fps);images.append(snaps)
import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
fig,axs=plt.subplots(2,3,figsize=(15,7))
for r in range(2):
 for c in range(3):axs[r,c].imshow(images[r][c]);axs[r,c].set_title(f'{list(videos)[r]}: frame{[0,160,320][c]}');axs[r,c].axis('off')
fig.tight_layout();fig.savefig(H/'dense_adapter_actual_collection_snapshots.png',dpi=120);plt.close(fig)
physics=np.load(out/'physics.npz');assert len(physics['simulation_time_s'])==1601
contacts=json.loads((out/'physics_contacts.json').read_text());result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),passed=True,sha_checks=n,videos=videos,frames=65,capture=321,physics=1601,reset_once=True,labels_not_decoded=True,source_only_collection_no_model_or_nav=True,contact_records=len(contacts),whole_research_complete=False)
(H/'dense_adapter_collection_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
