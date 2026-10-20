"""Full old whole-sequence clock regression only: no GT or fresh admission."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from partial_cue_source import PartialCueSetSource as Original
from partial_cue_source_v2 import PartialCueSetSource as Corrected
H=Path(__file__).resolve().parent;D=H/'controlled_heading_probe_run_v1';F=H/'partial_set_fresh_run_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();fields=0;error=None
for D in [D,F]:
 model=Corrected(.2);old=Original(.2);op=json.loads((D/'source_opportunity_records.json').read_text());start=np.load(D/'policy_inputs/frame_000.npz')['position_world_xy'];frames=sorted(int(f.stem.split('_')[-1]) for f in (D/'sensor_packets').glob('source_*.npz'));done=[];max_delta=0
 for i in frames:
  p=dict(np.load(D/'sensor_packets'/f'source_{i:03d}.npz'));t=float(p['time_s']);root=np.load(D/'known_poses'/f'frame_{i:03d}.npz')
  # Failed acquisition saved RGB/poses/masks at45 before incomplete policy capture.
  states=json.loads((D/'states.json').read_text());position=np.array(states[i]['position'])[:2];rgb=dict(np.load(D/'online_inputs'/f'frame_{i:03d}.npz'));mask=dict(np.load(D/'masks'/f'frame_{i:03d}.npz'));q=op[i//5]['qualified_seed']
  out=model.update(p,rgb,root,mask,q,t,i,position,np.array([2.2,0.]),start);max_delta=max(max_delta,abs(i*.1-t));done.append(i)
  if i<45:
   orig=old.update(p,rgb,root,mask,q,t,i,position,np.array([2.2,0.]),start)
   for k in out:np.testing.assert_equal(orig[k],out[k]);fields+=1
  elif error is None:
   try:old.update(p,rgb,root,mask,q,t,i,position,np.array([2.2,0.]),start)
   except ValueError as e:error=str(e)
   else:raise AssertionError('original failure not reproduced')
 result=dict(epochs=done,max_frame_time_drift_s=max_delta,renderer_GT_read=False,corrected_interface_all_epochs=True,not_new_actual_capture=True)
 if D==F:partial=result
 else:complete=result
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),historical_complete8s=complete,incomplete_fresh_prefix=partial,original_reproduced_error=error,pre_failure_fields_behavior_identical=fields,clock_fix_only_seconds_tolerance_1e5=True,no_GT_or_model_refit=True,not_fresh_validation=True,whole_research_complete=False,source_sha256={str(H/f):sha(H/f) for f in ['partial_cue_source.py','partial_cue_source_v2.py',Path(__file__).name]})
(H/'partial_cue_clock_complete_history_verification_v1.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
