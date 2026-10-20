"""Post-manifest endpoint risk and actual-trajectory support labels ONLY."""
from pathlib import Path
import json,datetime,hashlib,numpy as np
from point_support_shadow import swept_indices,support_result
from stereo_geometry import FOCAL
H=Path(__file__).resolve().parent;out=H/'dense_source_memory_runs';manifest=json.loads((out/'manifest.json').read_text());assert manifest['labels_not_loaded'] and manifest['future_trajectory_not_loaded'] and len(manifest['rows'])==130
sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();p=json.loads((H/'dense_source_memory_protocol.json').read_text());origin=np.array(p['origin']);shape=tuple(p['shape'])
for r in manifest['rows']:
 for a,b in [('path','sha256'),('packet_path','packet_sha256'),('maps_path','maps_sha256')]:assert sha(r[a])==r[b],r[a]
states=json.loads((H/'dense_adapter_heldout_run/states.json').read_text());assert len(states)==321
trajectory=np.array([s['position'][:2] for s in states])[60:];cells=swept_indices(trajectory,origin,.1,.7);np.savez_compressed(out/'offline_actual_sweep.npz',xy=trajectory,cells=cells,trajectory_is_offline_label_not_plan=True)
source_labels={};rows=[];summaries={}
for model in ('b0','adapter'):
 labelled={};alive={};counts=dict(points=0,floor_proxy_points=0,floor_proxy_labelled=0,floor_proxy_wrong_height=0,floor_proxy_wrong_3d=0,floor_proxy_unlabelled=0,floor_proxy_correct=0);summary=dict(source_endpoint_counts=counts,peak_floor_cells=0,peak_history_floor_cells=0,minimum_sweep_unknown=None,current_disk_passes=0,full_sweep_passes=0)
 for frame in range(0,321,5):
  record=json.loads((out/model/f'frame_{frame:03d}.json').read_text());packet=np.load(out/model/f'packet_{frame:03d}.npz');raw=np.load(H/'dense_adapter_heldout_run/frames'/f'frame_{frame:03d}.npz');t=record['time_s'];ids=packet['query_pixel_id'];y=ids//640;x=ids%640;z=raw['left_depth_m'][y,x];pose=raw['left_camera_usd_world'];rays=np.column_stack([(x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(len(x))])
  with np.errstate(invalid='ignore'):truth=pose[:3,3]+(rays*z[:,None])@(pose[:3,:3]@np.diag([1,-1,-1])).T
  valid=np.isfinite(truth).all(1)&(z>0);floor=(np.abs(packet['world'][:,2])+packet['sigma_m']<=.05)&(packet['sigma_m']<=.05);correctheight=valid&(np.abs(truth[:,2])<=.05);err=np.linalg.norm(truth-packet['world'],axis=1);correct=correctheight&(err<=.1)
  counts['points']+=len(ids);counts['floor_proxy_points']+=int(floor.sum());counts['floor_proxy_labelled']+=int((floor&valid).sum());counts['floor_proxy_wrong_height']+=int((floor&valid&~correctheight).sum());counts['floor_proxy_wrong_3d']+=int((floor&valid&(err>.1)).sum());counts['floor_proxy_unlabelled']+=int((floor&~valid).sum());counts['floor_proxy_correct']+=int((floor&correct).sum())
  np.savez_compressed(out/model/f'endpoint_labels_{frame:03d}.npz',query_pixel_id=ids,gt_world=truth,error_m=err,label_available=valid,floor_proxy=floor,correct_height=correctheight,correct_within_0p1=correct)
  maps=np.load(out/model/f'maps_{frame:03d}.npz');row=dict(model=model,frame=frame,time_s=t,arms={})
  for arm in ('current','history'):
   labels=maps[arm+'_labels'];whole=support_result(labels,cells);disk=record['arms'][arm]['current_disk'];count=int((labels==0).sum());history=int((maps[arm+'_provenance']==2).sum());row['arms'][arm]=dict(full_actual_sweep=whole,current_disk=disk,floor_cells=count,historical_floor_cells=history,source_rejected=True,no_action_authorized=True)
   if arm=='history':
    summary['peak_floor_cells']=max(summary['peak_floor_cells'],count);summary['peak_history_floor_cells']=max(summary['peak_history_floor_cells'],history);summary['minimum_sweep_unknown']=whole['unknown_cells'] if summary['minimum_sweep_unknown'] is None else min(summary['minimum_sweep_unknown'],whole['unknown_cells']);summary['current_disk_passes']+=int(disk['support_passed']);summary['full_sweep_passes']+=int(whole['support_passed'])
  rows.append(row)
 summary['startup_t6']=next(r for r in rows if r['model']==model and r['frame']==60)['arms'];summaries[model]=summary
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),maps_saved_utc=manifest['saved_utc'],rows=rows,summary=summaries,sweep_cells=len(cells),source_gate_preserved_rejected=True,consumer_supported=False,actual_future_trajectory_read_ONLY_postmap_foroffline_support=True,scope='endpoint/raster proxies, notfull3D physicalcollision or free certificate',whole_research_complete=False)
(out/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(dict(summary=summaries,sweep_cells=len(cells)),indent=2))
