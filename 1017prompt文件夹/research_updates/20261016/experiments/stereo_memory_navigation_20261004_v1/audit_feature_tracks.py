"""Post-prediction source-quality / frozen rejection-stage instrumentation."""
from pathlib import Path
import json,datetime
import numpy as np
from feature_tracks import causal_sources,match_descriptors,triangulate
from assess_feature_tracks import label,K
H=Path(__file__).resolve().parent;o=H/'feature_tracks_runs'
p=json.loads((H/'feature_tracks_protocol.json').read_text());a=json.loads((o/'assessment.json').read_text());g=json.loads((H/'matte_coded_board_run/board_geometry.json').read_text());rows=[];pair_labels=[]
for seq,samples in p['sequences'].items():
 history=[];sample_by_frame={s['frame']:s for s in samples}
 for s in samples:
  i=s['frame'];f=np.load(o/seq/f'features_{i:03d}.npz');record=json.loads((o/seq/f'prediction_{i:03d}.json').read_text());pair=[]
  for eye in ['left','right']:
   pair.append(dict(frame=i,eye=eye,time_s=float(f['time_s']),**{k:f[eye+'_'+k] for k in ['xy','desc','size','pose']}))
  for arm,sources in [('current',[pair[1]]),('causal',[pair[1]]+history)]:
   count=dict(descriptor_pairs=0,triangulation_pairs=0,sensitivity_pairs=0);q=set();qt=set();qs=set()
   for src in causal_sources(pair[0],sources):
    for qi,sj in match_descriptors(pair[0]['xy'],pair[0]['desc'],src['xy'],src['desc'],pair[0]['pose'],src['pose'],K):
     count['descriptor_pairs']+=1;q.add(qi);r=triangulate(pair[0]['xy'][qi],src['xy'][sj],pair[0]['pose'],src['pose'],K)
     if r is None:continue
     count['triangulation_pairs']+=1;qt.add(qi)
     if r['sigma_m']<=.1:count['sensitivity_pairs']+=1;qs.add(qi)
   count.update(descriptor_queries=len(q),triangulation_queries=len(qt),sensitivity_queries=len(qs),accepted_queries=len(record[arm]['points']),conflicting_queries=record[arm]['conflicting_queries'])
   rows.append(dict(sequence=seq,frame=i,arm=arm,left_features=len(pair[0]['xy']),right_features=len(pair[1]['xy']),**count))
   for pt in record[arm]['points']:
    sf,eye,si,st=pt['source'];source=np.load(o/seq/f'features_{sf:03d}.npz');raw=np.load(sample_by_frame[sf]['pose_file']);truth,kind=label(source[eye+'_xy'][si],raw,eye,g if seq=='matte_board' else None)
    query=next(v for v in a['points'] if (v['sequence'],v['frame'],v['arm'],v['index'])==(seq,i,arm,pt['query'][2]))
    mismatch=None if truth is None or query['label_world'] is None else float(np.linalg.norm(truth-query['label_world']))
    pair_labels.append(dict(sequence=seq,frame=i,arm=arm,query=pt['query'],source=pt['source'],error_m=query['error_m'],source_label_kind=kind,true_query_source_distance_m=mismatch,sigma_m=pt['sigma_m'],parallax_deg=pt['parallax_deg'],reprojection_px=pt['reprojection_px']))
  history=(history+pair)[-6:]
result=dict(audited_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=rows,accepted_source_labels=pair_labels,scope='offline nearest-pixel/plane labels AFTER complete predictions; wrong correspondence evidence only for labelled pair, no gate/source-selection changes')
(o/'source_audit.json').write_text(json.dumps(result,indent=2)+'\n')
for seq in p['sequences']:
 for arm in ['current','causal']:
  rs=[r for r in rows if r['sequence']==seq and r['arm']==arm];pairs=[r for r in pair_labels if r['sequence']==seq and r['arm']==arm];keys=['descriptor_queries','triangulation_queries','sensitivity_queries','accepted_queries','conflicting_queries']
  print(seq,arm,{k:sum(r[k] for r in rs) for k in keys},'labelled_source_mismatch>.1',sum(r['true_query_source_distance_m'] is not None and r['true_query_source_distance_m']>.1 for r in pairs))
