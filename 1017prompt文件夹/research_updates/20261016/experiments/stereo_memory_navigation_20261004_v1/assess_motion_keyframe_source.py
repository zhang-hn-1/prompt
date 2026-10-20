"""Independent labels only after the complete eight predictions are sealed."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from assess_feature_tracks import label
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    p=json.loads((H/'motion_keyframe_protocol.json').read_text());out=H/'motion_keyframe_runs'
    m=json.loads((out/'manifest.json').read_text());assert m['labels_not_loaded'] and len(m['rows'])==8
    expected={(seq,r['frame']) for seq,g in p['groups'].items() for r in g['samples']};assert {(r['sequence'],r['frame']) for r in m['rows']}==expected
    for key in ['source_sha256','input_sha256']:
        for f,s in p[key].items():assert sha(f)==s,f
    for r in m['rows']:assert sha(r['path'])==r['sha256']
    totals={};rows=[];point_rows=[]
    for seq,group in p['groups'].items():
        totals[seq]={arm:dict(queries=0,retained=0,correct=0,labelled_queries=0,labelled_retained=0,past=0,errors=[]) for arm in ['current','causal']}
        for sample in group['samples']:
            i=sample['frame'];record=json.loads((out/seq/f'prediction_{i:03d}.json').read_text());truth_file=Path(group['source_folder'])/'offline_truth'/f'frame_{i:03d}.npz'
            with np.load(truth_file) as truth:labels=[label(x,truth,'left') for x in record['current']['query_xy']]
            for arm in ['current','causal']:
                pred=record[arm];assert pred['query_xy']==record['current']['query_xy'];s=totals[seq][arm];errors=[];correct=0;past=0
                for pt in pred['points']:
                    index=pt['query'][2];world,kind=labels[index];err=None if world is None else float(np.linalg.norm(np.asarray(pt['world'])-world));ok=err is not None and err<=.1;correct+=ok;past+=pt['source'][3]<record['time_s']
                    if err is not None:errors.append(err)
                    point_rows.append(dict(sequence=seq,frame=i,arm=arm,index=index,error_m=err,correct=ok,kind=kind,query=pt['query'],source=pt['source'],world=pt['world'],label_world=None if world is None else world.tolist()))
                s['queries']+=len(labels);s['labelled_queries']+=sum(w is not None for w,k in labels);s['retained']+=len(pred['points']);s['correct']+=int(correct);s['labelled_retained']+=len(errors);s['past']+=int(past);s['errors'].extend(errors)
                rows.append(dict(sequence=seq,frame=i,arm=arm,queries=len(labels),retained=len(pred['points']),correct=int(correct),p90_m=float(np.percentile(errors,90)) if errors else None,seconds_both_arms=record['seconds'],past=int(past),query_floor_labels=sum(k=='floor' for w,k in labels),retained_floor_labels=sum(pt['kind']=='floor' for pt in point_rows if pt['sequence']==seq and pt['frame']==i and pt['arm']==arm)))
        for arm,s in totals[seq].items():
            errors=s.pop('errors');s['all_query_fraction']=s['correct']/s['queries'] if s['queries'] else 0.;s['retained_precision']=s['correct']/s['retained'] if s['retained'] else None;s['p90_m']=float(np.percentile(errors,90)) if errors else None
            s['gate_pass']=bool(s['queries']>=100 and s['all_query_fraction']>=.9 and s['retained_precision'] is not None and s['retained_precision']>=.99 and s['p90_m'] is not None and s['p90_m']<=.1)
    r=dict(assessed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),predictions_saved_utc=m['saved_utc'],protocol_sha256=sha(H/'motion_keyframe_protocol.json'),manifest_sha256=sha(out/'manifest.json'),sequences=totals,rows=rows,points=point_rows,candidate_pass=all(v['causal']['gate_pass'] for v in totals.values()),fresh=False,estimated_pose=False,dense_depth=False,certified_free=False,formal_e0e1e2=False,whole_research_complete=False,label_scope='knownpose development; nearestpixel renderer axialdepth plus subpixel query ray, floorheight proxy not body-ID')
    (out/'assessment.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n');print(json.dumps(totals,indent=2))
if __name__=='__main__':main()
