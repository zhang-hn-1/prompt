"""One archive privileged-current-depth counterfactual; no future trajectory input."""
from pathlib import Path
import json,datetime,hashlib,numpy as np
from perfect_visible_points import perfect_packet
from dense_source_points import records
from point_support_shadow import PointSupportMap,swept_indices,support_result
from startup_prior import ImmutableStartupPrior
H=Path(__file__).resolve().parent;sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
p=json.loads((H/'startup_prior_protocol.json').read_text())
for kind in ('source_sha256','input_sha256'):
 for f,d in p[kind].items():assert sha(f)==d,f
out=H/'startup_prior_runs';out.mkdir(exist_ok=False);masks=np.load(p['mask']);origin=p['origin'];shape=tuple(p['shape']);history=PointSupportMap(origin,shape,.1);prior=None;rows=[];start=now();(H/'startup_prior_status.json').write_text(json.dumps(dict(status='running',start_utc=start))+'\n')
for frame in range(0,321,5):
 raw=np.load(H/'dense_adapter_heldout_run/frames'/f'frame_{frame:03d}.npz');current={k:raw[k] for k in ['left_depth_m','right_depth_m','left_camera_usd_world','right_camera_usd_world','root_position_world_m','time_s']}
 packet=perfect_packet(current,masks['left'],masks['right'],frame);t=packet['time_s'];disk=swept_indices([current['root_position_world_m'][:2]],origin,.1,.7)
 if frame==60:
  assert len(disk)==185;prior=ImmutableStartupPrior(disk,t,source='task_assumption',trusted=True);np.savez_compressed(out/'immutable_prior.npz',cells=disk,time_s=t,root_xy=current['root_position_world_m'][:2],is_observation=False)
 history.update(records(packet),t,0.);s=history.snapshot(t);pp=out/f'packet_{frame:03d}.npz';np.savez_compressed(pp,**packet);mp=out/f'map_{frame:03d}.npz';np.savez_compressed(mp,**{k:s[k] for k in ('labels','provenance','floor_time','hazard_time')},current_disk=disk)
 r=dict(frame=frame,time_s=t,points=len(packet['world']),records=len(s['records']),ignored=s['ignored_records'],conflicts=s['conflict_cells'],floor_cells=int((s['labels']==0).sum()),hazard_cells=int((s['labels']==1).sum()),cold=support_result(s['labels'],disk),with_prior=None if prior is None else prior.evaluate(s['labels'],disk,t),source='privileged_renderer_current_binocular_visible',knownpose=True,model_imputed_regions=0,action_authorized=False)
 rp=out/f'frame_{frame:03d}.json';rp.write_text(json.dumps(r,indent=2)+'\n');rows.append(dict(frame=frame,time_s=t,path=str(rp),sha256=sha(rp),packet_path=str(pp),packet_sha256=sha(pp),map_path=str(mp),map_sha256=sha(mp)))
 if frame%50==0:print(frame,r['points'],r['floor_cells'],r['cold']['unknown_cells'],flush=True)
(out/'manifest.json').write_text(json.dumps(dict(saved_utc=now(),rows=rows,future_trajectory_not_read=True,privileged_source=True,prior_is_observation=False,scope='conditional archived necessary support ONLY'),indent=2)+'\n');(H/'startup_prior_status.json').write_text(json.dumps(dict(status='finished',start_utc=start,end_utc=now(),maps=65),indent=2)+'\n')
