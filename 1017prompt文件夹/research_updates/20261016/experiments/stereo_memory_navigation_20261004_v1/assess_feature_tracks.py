"""Offline labels only, after the complete frozen prediction manifest exists."""
from pathlib import Path
import datetime, hashlib, json
import numpy as np
from feature_tracks import rotation
H=Path(__file__).resolve().parent
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])

def label(xy,raw,eye,board=None):
    x,y=np.rint(xy).astype(int)
    if not (0<=x<640 and 0<=y<480):return None,'out_of_bounds'
    depth=float(raw[eye+'_depth_m'][y,x]);pose=raw[eye+'_camera_usd_world']
    ray=rotation(pose)@np.linalg.solve(K,np.r_[xy,1]);centre=pose[:3,3]
    if not np.isfinite(depth) or depth<=0:return None,'no_renderer_label'
    if board is not None:
        if abs(ray[0])<1e-12:return None,'no_board_ray'
        d=(board['plane_x']-centre[0])/ray[0];point=centre+ray*d
        centres=np.array(board['centres']);extra=.27
        lo=centres.min(0);hi=centres.max(0)
        if d<=0 or not (lo[1]-extra<=point[1]<=hi[1]+extra and lo[2]-.18<=point[2]<=hi[2]+.36):return None,'outside_board_label'
        if abs(depth-d)>.01:return None,'board_renderer_disagrees'
        return point,'board_plane'
    point=centre+ray*depth
    return point,'floor' if point[2]<.08 else 'elevated_or_other'

def main():
    p=json.loads((H/'feature_tracks_protocol.json').read_text());folder=H/'feature_tracks_runs'
    manifest=json.loads((folder/'manifest.json').read_text());assert manifest['labels_not_loaded']
    for f,s in p['source_sha256'].items():assert sha(f)==s,f
    for r in manifest['rows']:assert sha(r['path'])==r['sha256']
    assert len(manifest['rows'])==26 and manifest['saved_utc']>p['sealed_utc']
    board=json.loads((H/'matte_coded_board_run/board_geometry.json').read_text())
    rows=[];point_rows=[];totals={};memory_rows=[];all_labels={}
    for seq,samples in p['sequences'].items():
        sums={arm:dict(queries=0,retained=0,correct=0,labelled_queries=0,labelled_retained=0,errors=[],past=0,conflicts=0,partitions={}) for arm in ['current','causal']}
        for sample in samples:
            i=sample['frame'];record=json.loads((folder/seq/f'prediction_{i:03d}.json').read_text());raw=np.load(sample['pose_file'])
            labels=[label(xy,raw,'left',board if seq=='matte_board' else None) for xy in record['current']['query_xy']]
            all_labels[(seq,i)]=labels
            arm_correct={}
            for arm in ['current','causal']:
                pred=record[arm];assert pred['query_xy']==record['current']['query_xy'];s=sums[arm]
                s['queries']+=pred['query_count'];s['labelled_queries']+=sum(point is not None for point,kind in labels)
                s['retained']+=len(pred['points']);s['conflicts']+=pred['conflicting_queries'];errors=[];correct=set();past=0
                for pt in pred['points']:
                    index=pt['query'][2];point,kind=labels[index];err=None if point is None else float(np.linalg.norm(np.array(pt['world'])-point))
                    ok=err is not None and err<=.1;is_past=pt['source'][3]<record['time_s'];past+=is_past
                    if err is not None:errors.append(err);s['errors'].append(err);s['labelled_retained']+=1
                    if ok:correct.add(index);s['correct']+=1
                    part=s['partitions'].setdefault(kind,dict(retained=0,correct=0));part['retained']+=1;part['correct']+=int(ok)
                    point_rows.append(dict(sequence=seq,frame=i,arm=arm,index=index,error_m=err,correct=ok,label_kind=kind,source=pt['source'],past=bool(is_past),world=pt['world'],label_world=None if point is None else point.tolist()))
                s['past']+=past;arm_correct[arm]=correct
                counts={}
                for pt,kind in labels:counts[kind]=counts.get(kind,0)+1
                rows.append(dict(sequence=seq,frame=i,arm=arm,queries=pred['query_count'],label_kinds=counts,retained=len(pred['points']),correct=len(correct),past=past,conflicts=pred['conflicting_queries'],p90_m=None if not errors else float(np.percentile(errors,90)),seconds_both_arms_plus_extract=record['seconds']))
            rows[-1]['correct_added_vs_current']=len(arm_correct['causal']-arm_correct['current']);rows[-1]['correct_lost_vs_current']=len(arm_correct['current']-arm_correct['causal'])
            memory_rows.append(dict(sequence=seq,frame=i,time_s=record['time_s'],records=len(record['memory']),older_queries=sum(pt['query'][3]<record['time_s'] for pt in record['memory']),unique_query_provenance=len({tuple(pt['query']) for pt in record['memory']})))
        for arm,s in sums.items():
            e=s.pop('errors');s['unconditional_fraction']=s['correct']/s['queries'] if s['queries'] else 0.;s['conditional_fraction']=s['correct']/s['retained'] if s['retained'] else None;s['p90_m']=None if not e else float(np.percentile(e,90))
            s['gate_pass']=bool(s['queries']>=100 and s['unconditional_fraction']>=.9 and s['p90_m'] is not None and s['p90_m']<=.1)
        totals[seq]=sums
    # Memory records are checked against their original query label, never the current image.
    for row in memory_rows:
        record=json.loads((folder/row['sequence']/f"prediction_{row['frame']:03d}.json").read_text());valid=correct=0
        for pt in record['memory']:
            q=pt['query'];truth,kind=all_labels[(row['sequence'],q[0])][q[2]]
            if truth is not None:valid+=1;correct+=np.linalg.norm(np.array(pt['world'])-truth)<=.1
        row.update(labelled_records=valid,correct_records=int(correct),certified_free=False)
    result=dict(assessed_utc=now(),predictions_saved_utc=manifest['saved_utc'],protocol_sha256=sha(H/'feature_tracks_protocol.json'),manifest_sha256=sha(folder/'manifest.json'),sequences=totals,rows=rows,points=point_rows,memory_rows=memory_rows,candidate_accepted=all(s['causal']['gate_pass'] for s in totals.values()),scope='known-pose archived development; original renderer nearest pixel axial depth along query subpixel ray; board bounded plane checked against renderer; no unique spatial area/free-space certification')
    (folder/'assessment.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ['rows','points','memory_rows']},indent=2))
if __name__=='__main__':main()
