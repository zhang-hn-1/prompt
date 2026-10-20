"""Three recorded jobs; wall-based replay client clock, no scene/physics/GT."""
from pathlib import Path
import time,json,hashlib,traceback,datetime
import numpy as np
from async_stereo_worker import AsyncStereoWorker
H=Path(__file__).resolve().parent;D=H/'live_paired_texture_navigation_run_v1';O=H/'spawn_stereo_transport_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 config=json.loads((H/'spawn_stereo_transport_protocol_v1.json').read_text())
 for group in ('source_sha256','input_sha256'):
  for f,s in config[group].items():assert sha(f)==s,f
 assert config['frames']==[0,150,295] and config['maximum_wall_s']==300
 O.mkdir(exist_ok=False);start=time.monotonic();rows=[];submit_times=[];main_iterations=0;ok=False;w=None
 try:
  jobs=[]
  for i in config['frames']:
   name=f'frame_{i:03d}.npz'
   with np.load(D/'source_inputs'/name) as z:rgb={k:z[k].copy() for k in ('left_rgb','right_rgb')}
   with np.load(D/'estimated_poses'/name) as z:pose={k:z[k].copy() for k in ('left_camera_usd_world','right_camera_usd_world','root_position_world_m')}
   with np.load(D/'history'/name) as z:history=z['proprio_history'].copy()
   history[:,:9]=0.
   with np.load(D/'sensor_packets'/f'source_{i:03d}.npz') as z:birth=float(z['time_s'])
   inputs=[D/'source_inputs'/name,D/'estimated_poses'/name,D/'history'/name]
   jobs.append(dict(frame=i,time_s=birth,**rgb,pose=pose,history=history,input_sha256={str(f.resolve()):sha(f) for f in inputs}))
  # No simulated current time: all three archived exposures precede replay clock.
  base=max(j['time_s'] for j in jobs);clock=lambda:base+time.monotonic()-start
  w=AsyncStereoWorker('frozen_stereo_worker_backend','Backend');next_job=0
  while len(rows)<3:
   if time.monotonic()-start>300:raise TimeoutError('predeclared transport budget')
   if next_job==len(rows) and next_job<3:
    j=jobs[next_job];begin=time.monotonic();w.submit(j,frame=j['frame'],birth_time_s=j['time_s'],now=clock());submit_times.append(time.monotonic()-begin);next_job+=1
   for delivered in w.poll(now=clock()):
    p=delivered['packet'];i=p['frame'];name=f'frame_{i:03d}.npz'
    with np.load(D/'source_predictions'/name) as old:
     for k in old.files:
      if k in p['prediction']:np.testing.assert_array_equal(p['prediction'][k],old[k])
    with np.load(D/'sensor_packets'/f'source_{i:03d}.npz') as old:
     for k in old.files:np.testing.assert_array_equal(p['raw'][k],old[k])
    with np.load(D/'partial_observations'/name) as old:
     for k in ('world','sigma_m','query_pixel_id','partner_pixel_id'):np.testing.assert_array_equal(p['surface'][k],old[k])
    assert p['input_sha256']==jobs[len(rows)]['input_sha256']
    np.savez_compressed(O/name,**p['prediction'],**{'surface_'+k:p['surface'][k] for k in ('world','sigma_m','query_pixel_id','partner_pixel_id')},frame=i,time_s=p['time_s'],available_time_s=delivered['available_time_s'])
    rows.append(dict(frame=i,birth_time_s=p['time_s'],available_replay_time_s=delivered['available_time_s'],timing=p['timing'],NN_and_original_CAD_and_selected_points_exact=True,input_sha256=p['input_sha256']))
   main_iterations+=1;time.sleep(.005)
  ok=True
 except BaseException:
  (O/'failure.txt').write_text(traceback.format_exc());raise
 finally:
  if w:w.close()
  status=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),complete=ok,rows=rows,wall_s=time.monotonic()-start,main_loop_iterations=main_iterations,submit_wall_s=submit_times,worker_pid=None if w is None else w.pid,worker_alive_after_close=False if w is None else w.is_alive,trace=[] if w is None else w.trace,dropped_frames=[] if w is None else w.dropped_frames,replay_clock_not_physics=True,no_simulation_or_GT_actor=True,no_navigation_or_realtime_claim=True,whole_complete=False)
  (O/'assessment.json').write_text(json.dumps(status,indent=2)+'\n');(O/'artifact_sha256.json').write_text(json.dumps({str(f.resolve()):sha(f) for f in O.rglob('*') if f.is_file()},indent=2)+'\n')
  print(json.dumps(status,indent=2),flush=True)
if __name__=='__main__':main()
