"""Offline assessment AFTER complete live predictions; no control/model admission."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
import cv2
from assess_feature_tracks import label
from route_geometry_audit import project
from point_support_shadow import swept_indices,support_result
H=Path(__file__).resolve().parent;O=H/'oracle_rgb_shadow_run';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=json.loads((H/'oracle_rgb_shadow_protocol.json').read_text());status=json.loads((H/'oracle_rgb_shadow_status.json').read_text());assert status['exit_code']==0 and p['sealed_utc']<status['start_utc']
 manifest=json.loads((O/'artifact_manifest.json').read_text())
 for f,s in manifest['artifact_sha256'].items():assert sha(f)==s,f
 for f,s in p['source_sha256'].items():assert sha(f)==s,f
 metrics=json.loads((O/'metrics.json').read_text());identity=json.loads((O/'identity.json').read_text());N=identity['captures'];assert N==identity['policy_calls']+1
 ref=np.load(H/'oracle_rgb_shadow_reference.npz')['pose'][:,:2];cells=swept_indices(ref,[-20,-20],.1,.7);arrays=np.load(O/'episode_arrays.npz');progress,lateral,_=project(arrays['positions_world_xy'],ref)
 stats={k:dict(queries=0,retained=0,correct=0,labelled_queries=0,labelled_retained=0,past=0,errors=[]) for k in ['current','causal']};rows=[];labels=[];memory=[];maprows=[]
 for i in range(N):
  r=json.loads((O/'shadow'/f'prediction_{i:03d}.json').read_text());raw=np.load(O/'offline_truth'/f'frame_{i:03d}.npz');labels.append([label(xy,raw,'left') for xy in r['current']['query_xy']]);row=dict(frame=i,time_s=r['time_s'],feature_seconds=r['seconds'],capture_wall_s=r['capture_wall_s'])
  for arm in ['current','causal']:
   a=r[arm];s=stats[arm];assert a['query_count']==len(labels[-1]);s['queries']+=a['query_count'];s['retained']+=len(a['points']);s['labelled_queries']+=sum(x is not None for x,k in labels[-1]);correct=0
   for pt in a['points']:
    truth,kind=labels[-1][pt['query'][2]];error=None if truth is None else float(np.linalg.norm(np.array(pt['world'])-truth));ok=error is not None and error<=.1
    s['correct']+=ok;correct+=ok;s['past']+=pt['source'][3]<r['time_s']
    if error is not None:s['errors'].append(error);s['labelled_retained']+=1
   row[arm]=dict(queries=a['query_count'],retained=len(a['points']),correct=int(correct))
  valid=correct=0
  for pt in r['records']:
   truth,_=labels[pt['query'][0]][pt['query'][2]]
   if truth is not None:valid+=1;correct+=np.linalg.norm(np.array(pt['world'])-truth)<=.1
  memory.append(dict(frame=i,records=len(r['records']),labelled=valid,correct=int(correct),wrong=int(valid-correct),unlabelled=len(r['records'])-valid,older=sum(pt['query'][3]<r['time_s'] for pt in r['records'])))
  grid=np.load(O/'shadow'/f'map_{i:03d}.npz');support=support_result(grid['labels'],cells)
  maprows.append(dict(frame=i,floor_cells=r['floor_cells'],hazard_cells=r['hazard_cells'],support={k:v for k,v in support.items() if not k.endswith('_indices')},certified_free=False));rows.append(row)
 for arm,s in stats.items():
  errors=s.pop('errors');s['unconditional_fraction']=s['correct']/s['queries'] if s['queries'] else 0.;s['p90_m']=float(np.percentile(errors,90)) if errors else None;s['necessary_historical_gate_pass']=bool(s['queries']>=100 and s['unconditional_fraction']>=.9 and s['p90_m'] is not None and s['p90_m']<=.1)
 videos=[]
 for name in ['stereo_view','third_person']:
  cap=cv2.VideoCapture(str(O/(name+'.mp4')));frames=0
  while cap.read()[0]:frames+=1
  fps=cap.get(cv2.CAP_PROP_FPS);cap.release();assert frames==N and fps==10.;videos.append(dict(name=name,frames=frames,fps=fps))
 clocks=json.loads((O/'camera_clocks.json').read_text());ph=np.load(O/'physics.npz');contacts=json.loads((O/'physics_contacts.json').read_text());reference_pathlength=np.linalg.norm(np.diff(ref,axis=0),axis=1).sum()
 output=dict(assessed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),all_live_predictions_saved_utc=manifest['saved_utc'],navigation={k:v for k,v in metrics.items() if not isinstance(v,list)},reference=dict(cached_waypoints=len(ref),declared_path_length_m=p['reference_length_m'],polyline_length_m=float(reference_pathlength),actual_along_final_m=float(progress[-1]),actual_along_max_m=float(progress.max()),lateral_rms_m=float(np.sqrt(np.mean(lateral**2))),lateral_peak_m=float(lateral.max()),evaluation_independent_of_policy_index=True),source_stats=stats,memory_rows=memory,map_rows=maprows,sweep_cells=len(cells),source_rows=rows,acquisition=dict(captures=N,policy_calls=identity['policy_calls'],physics_samples=len(ph['simulation_time_s']),contact_samples=len(contacts),actual_contact_samples=sum(bool(x['contact']) for x in contacts),max_clock_age_s=max(abs(x['raw_time_s']-t) for x in clocks for t in x['eyes'].values()),video=videos),latency=dict(feature_p50_s=float(np.percentile([r['feature_seconds'] for r in rows],50)),feature_p95_s=float(np.percentile([r['feature_seconds'] for r in rows],95)),capture_p50_s=float(np.percentile([r['capture_wall_s'] for r in rows],50)),capture_p95_s=float(np.percentile([r['capture_wall_s'] for r in rows],95)),physics_budget_not_wall_clock=True),source_admitted=False,consumer_admitted=False,formal_matrix=False,whole_research_complete=False,scope='ONE known-map Oracle full navigation diagnostic. Live frozen rejected ORB RGB+known-pose shadow, no actions. Current-query full denominator and limited nearest-pixel depth labels; no bodyID/wholepyramid/3D certificate/estimatedpose/learning.')
 (H/'oracle_rgb_shadow_assessment.json').write_text(json.dumps(output,indent=2)+'\n');print(json.dumps({k:v for k,v in output.items() if k not in ['source_rows','memory_rows','map_rows']},indent=2))
if __name__=='__main__':main()
