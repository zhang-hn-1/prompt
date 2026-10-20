"""One presealed causal feature structure; paired current/history input ablation."""
from pathlib import Path
import datetime,json,hashlib,time
import numpy as np
from feature_tracks import extract,predict_features,FeatureMemory
H=Path(__file__).resolve().parent;now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
def main():
 p=json.loads((H/'feature_tracks_protocol.json').read_text())
 for f,s in p['source_sha256'].items():assert sha(f)==s,f
 out=H/'feature_tracks_runs';out.mkdir(exist_ok=False);masks=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz');K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
 rows=[]
 for seq,samples in p['sequences'].items():
  folder=out/seq;folder.mkdir();history=[];memory=FeatureMemory(10)
  for sample in samples:
   raw=np.load(sample['rgb_file']);known=np.load(sample['pose_file']);t=float(raw[sample['time_field']]);assert t==float(known[sample['time_field']]);i=sample['frame'];pair=[];features={}
   start=time.perf_counter()
   for eye in ['left','right']:
    rgb=raw[eye+'_rgb'];xy,desc,size=extract(rgb,masks[eye]);pose=known[eye+'_camera_usd_world']
    pair.append(dict(xy=xy,desc=desc,size=size,pose=pose,frame=i,time_s=t,eye=eye))
    for key,value in [('xy',xy),('desc',desc),('size',size),('pose',pose)]:features[eye+'_'+key]=value
   fpath=folder/f'features_{i:03d}.npz';np.savez_compressed(fpath,**features,time_s=t,frame=i)
   current=predict_features(pair[0],[pair[1]],K);causal=predict_features(pair[0],[pair[1]]+history,K)
   memory.update(causal['points'],t,0.)
   record=dict(sequence=seq,frame=i,time_s=t,current=current,causal=causal,memory=memory.snapshot(t),pose_sigma_m=0.,seconds=time.perf_counter()-start,source_packet_fields=sorted(pair[0].keys()),features_file=str(fpath),features_sha256=sha(fpath),rgb_source=sample['rgb_file'],known_pose_source=sample['pose_file'])
   path=folder/f'prediction_{i:03d}.json';path.write_text(json.dumps(record,indent=2)+'\n')
   rows.append(dict(sequence=seq,frame=i,time_s=t,path=str(path),sha256=sha(path),current_queries=current['query_count'],current_retained=len(current['points']),causal_retained=len(causal['points']),memory_records=len(record['memory']),seconds=record['seconds']))
   history=(history+pair)[-6:]
   print(seq,i,current['query_count'],len(current['points']),len(causal['points']),flush=True)
 manifest=dict(saved_utc=now(),labels_not_loaded=True,rows=rows,scope='RGB actualdescriptors+knownposes only; whole-sequence grouped archiveddevelopment, no ID/layout/depth/future inputs',source_sha256={str(f):sha(f) for f in [Path(__file__),H/'feature_tracks.py',H/'feature_tracks_protocol.json']})
 (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print('FEATURE_PREDICTIONS_COMPLETE',len(rows),manifest['saved_utc'])
if __name__=='__main__':main()
