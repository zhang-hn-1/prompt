from pathlib import Path
import json,datetime,hashlib
import numpy as np
import cv2
from multiview_rgb import optical_rotation
from floor_adapter import apply_head
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 started=datetime.datetime.now(datetime.timezone.utc).isoformat();p=json.loads((H/'floor_adapter_fit_protocol.json').read_text());fit=json.loads((H/'floor_adapter_fit.json').read_text());a=json.loads((H/'floor_adapter_heldout_audit.json').read_text());assert not a['candidate_accepted'];m=json.loads((H/'floor_adapter_heldout_predictions/manifest.json').read_text());masks=np.load(p['mask_path']);par=p['projection'];yy,xx=np.mgrid[:480,:640];rays=np.stack([(xx-par['cx'])/par['focal'],(yy-par['cy'])/par['focal'],np.ones((480,640))],axis=-1);norm=np.linalg.norm(rays,axis=-1);rows=[];snapshots={};zs=[]
 for r in m['rows']:
  i=r['frame'];eye=r['eye'];z=np.load(H/'semantic_adapter_heldout_run/frames'/f'frame_{i:03d}.npz');pred=np.load(r['prediction']);d=z[eye+'_depth_m'];pose=z[eye+'_camera_usd_world'];rot=optical_rotation(pose);wr=rays@rot.T
  with np.errstate(invalid='ignore'):pts=pose[:3,3]+(rays*d[...,None])@rot.T
  finite=np.isfinite(d)&(d>0);floor=finite&~masks[eye]&(abs(pts[...,2])<=.05);query=floor&(d>=.5)&(d<=3);pd=pred['plane_axis_depth_m'];geom=np.isfinite(pd);k=2*par['patch_radius']+1;foot=cv2.boxFilter((~masks[eye]).astype(np.float32),-1,(k,k),normalize=True,borderType=cv2.BORDER_CONSTANT)>=1-1e-6
  err=abs(pd-d)*norm;within=np.isfinite(err)&(err<=.1);oracle=query&geom&foot&within;good=pred['confidence']&floor&within;false=pred['confidence']&~good;semantic=pred['semantic_floor_native']
  row=dict(frame=i,eye=eye,queries=int(query.sum()),lost_plane_geometry_or_ray=int((query&(~geom|~within)).sum()),lost_only_mask_footprint=int((query&geom&within&~foot).sum()),oracle_correct_ceiling=int(oracle.sum()),classifier_misses_within_ceiling=int((oracle&~semantic).sum()),actual_correct_queries=int((good&query).sum()),false_outputs=int(false.sum()),false_unavailable_truth=int((false&~finite).sum()),false_nonground_height=int((false&finite&(abs(pts[...,2])>.05)).sum()),false_floor_ray=int((false&floor).sum()))
  assert row['queries']==row['lost_plane_geometry_or_ray']+row['lost_only_mask_footprint']+row['oracle_correct_ceiling'];assert row['oracle_correct_ceiling']-row['classifier_misses_within_ceiling']==row['actual_correct_queries'];rows.append(row)
  if false.any():zs.extend(pts[...,2][false&finite].tolist())
  if i in [0,50,100,150,200,250,320]:
   rgb=z[eye+'_rgb'];overlay=rgb.copy();overlay[query]=(overlay[query]*.3+np.array([40,100,255])*.7).astype('uint8');overlay[good]=(overlay[good]*.3+np.array([40,230,50])*.7).astype('uint8');overlay[false]=(overlay[false]*.3+np.array([255,50,20])*.7).astype('uint8');snapshots[(i,eye)]=(rgb,overlay)
 zz=np.load(fit['parameters']);head=dict(weight=zz['weight'],bias=float(zz['bias']));train=[]
 for r in fit['rows']:
  z=np.load(r['file']);q=apply_head({'features':z['features']},head)>=.9;y=z['labels'].astype(bool);train.append(dict(frame=r['frame'],eye=r['eye'],samples=len(y),positives=int(y.sum()),predicted=int(q.sum()),correct_positive=int((q&y).sum()),false_positive=int((q&~y).sum())))
 sums={k:sum(r[k] for r in rows) for k in rows[0] if k not in ['frame','eye']};sums['oracle_ceiling_fraction']=sums['oracle_correct_ceiling']/sums['queries'];sums['classifier_missed_fraction_of_ceiling']=sums['classifier_misses_within_ceiling']/sums['oracle_correct_ceiling'];sums['false_height_quantiles_m']=np.percentile(zs,[0,25,50,75,100]).tolist();sums['rows_with_ceiling_90percent']=sum(r['queries']>0 and r['oracle_correct_ceiling']/r['queries']>=.9 for r in rows)
 from PIL import Image,ImageDraw
 canvas=Image.new('RGB',(1280,7*262),'white');draw=ImageDraw.Draw(canvas)
 for k,i in enumerate([0,50,100,150,200,250,320]):
  for e,eye in enumerate(['left','right']):
   rgb,overlay=snapshots[(i,eye)];canvas.paste(Image.fromarray(rgb).resize((320,240)),(e*640,k*262+22));canvas.paste(Image.fromarray(overlay).resize((320,240)),(e*640+320,k*262+22));draw.text((e*640,k*262+3),f'frame{i} {eye}: RGB | BLUE truth query GREEN correct RED wrong',fill='black')
 canvas.save(H/'floor_adapter_causal_snapshots_v2.png')
 import matplotlib
 matplotlib.use('Agg');import matplotlib.pyplot as plt
 fig,axs=plt.subplots(2,1,figsize=(10,7),sharex=True)
 for eye in ['left','right']:
  rr=[r for r in rows if r['eye']==eye];t=np.array([r['frame']/10 for r in rr]);n=np.array([r['queries'] for r in rr]);ceiling=np.divide([r['oracle_correct_ceiling'] for r in rr],n,out=np.zeros(len(rr)),where=n>0);actual=np.divide([r['actual_correct_queries'] for r in rr],n,out=np.zeros(len(rr)),where=n>0);axs[0].plot(t,ceiling,'--',label=eye+' unchanged geometry ceiling (GT diagnostic)');axs[0].plot(t,actual,label=eye+' actual correct fraction');axs[1].plot(t,[r['false_outputs'] for r in rr],label=eye+' wrong projected pixels')
 axs[0].axhline(.9,color='black',ls=':');axs[0].set_ylabel('All-query fraction (zero for no query)');axs[1].set_ylabel('Pixel count');axs[1].set_xlabel('Sequence time(s)');[ax.legend(fontsize=8) for ax in axs];fig.tight_layout();fig.savefig(H/'floor_adapter_causal_metrics_v2.png');plt.close(fig)
 videos=[];video_snap=Image.new('RGB',(1280,7*262),'white');vd=ImageDraw.Draw(video_snap)
 for eye in ['third_person','stereo_view']:
  file=H/'semantic_adapter_heldout_run'/(eye+'.mp4');assert file.exists();cap=cv2.VideoCapture(str(file));fps=cap.get(cv2.CAP_PROP_FPS);frames=0
  while True:
   ok,bgr=cap.read()
   if not ok:break
   if frames in [0,50,100,150,200,250,320]:
    k=[0,50,100,150,200,250,320].index(frames);x=0 if eye=='third_person' else 640;video_snap.paste(Image.fromarray(cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)).resize((640,240)),(x,k*262+22));vd.text((x,k*262+3),f'Actual video {eye} frame{frames}',fill='black')
   frames+=1
  cap.release();assert frames==321 and fps==10;videos.append(dict(file=str(file),sha256=sha(file),decoded_frames=frames,fps=fps))
 video_snap.save(H/'floor_adapter_actual_video_snapshots_v2.png')
 result=dict(started_utc=started,ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),protocol_sha256=sha(H/'floor_adapter_diagnostic_protocol.json'),rows=rows,summary=sums,training_rows=train,videos=videos,new_fits=0,new_inference=0,new_physics=0,candidate_still_rejected=True,whole_research_complete=False);(H/'floor_adapter_causal_diagnostic_v2.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(sums,flush=True)
if __name__=='__main__':main()
