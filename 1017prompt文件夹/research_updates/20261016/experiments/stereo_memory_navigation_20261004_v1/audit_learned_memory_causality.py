"""Actual-prefix reconstruction, truncation/future-payload rejection; no labels."""
from pathlib import Path
import numpy as np,torch,json,datetime,hashlib
from learned_cell_memory import CellMemory
from packet_scalars import normalize_npz_packet
from memory_reliability_head import ReliabilityHead
H=Path(__file__).resolve().parent;state=torch.load(H/'learned_memory_fit/Mmem1.pt',weights_only=True,map_location='cpu');model=ReliabilityHead(state['mean'].numpy(),state['scale'].numpy());model.load_state_dict(state);torch.set_num_threads(1)
class FutureBomb(dict):
 def __getitem__(self,key):
  if key!='time_s':raise AssertionError('future payload was read')
  return super().__getitem__(key)
results=[]
for kind in ['far','near']:
 case=f'memory_validation_{kind}_run';o=H/case;states=json.loads((o/'states.json').read_text());init=np.load(o/'maps/frame_000.npz');paths=sorted((o/'sensor_packets').glob('*.npz'));original=[];counts=[]
 for cut in [None,60]:
  m=CellMemory(init['origin'],init['labels'].shape,float(init['resolution']));out=[]
  for p in paths:
   d=np.load(p);i=int(d['frame']);t=float(d['time_s'])
   if cut is not None and i>cut:break
   packet=normalize_npz_packet({k:d[k] for k in d.files});rgb=np.load(o/'online_inputs'/f'frame_{i:03d}.npz');pose=np.load(o/'known_poses'/f'frame_{i:03d}.npz');hist=np.load(o/'history'/f'frame_{i:03d}.npz');mask=np.load(o/'masks'/f'frame_{i:03d}.npz');ctx=dict(time_s=t,left_rgb=rgb['left_rgb'],right_rgb=rgb['right_rgb'],left_mask=mask['left'],left_camera_position=pose['left_camera_usd_world'][:3,3],root_position=np.array(states[i]['position']),previous_request=hist['action_history'][-1]);m.ingest(packet,ctx,t);s=m.snapshot(t);score=model.predict(s['features'].astype(np.float32));saved=np.load(H/'learned_memory_held_predictions'/case/f'frame_{i:03d}.npz');feature=np.load(H/'learned_memory_held_features'/case/f'frame_{i:03d}.npz');np.testing.assert_equal(s['features'],feature['features']);np.testing.assert_equal(s['cells'],saved['cells']);np.testing.assert_equal(score,saved['scores']);np.testing.assert_equal(s['last_query_times'],saved['last_query_times']);assert all(q[3]<=t for group in s['query_ids'] for q in group);out.append((i,s['features'].copy(),score.copy()))
  counts.append(len(out))
  if cut is None:original=out
  else:
   for old,new in zip(original,out):assert old[0]==new[0];np.testing.assert_equal(old[1],new[1]);np.testing.assert_equal(old[2],new[2])
   before=m.snapshot(t)
   for replacement in [FutureBomb(time_s=t+1),FutureBomb(time_s=t+100)]:
    try:m.ingest(replacement,{},t)
    except ValueError:pass
    else:raise AssertionError('future packet admitted')
    after=m.snapshot(t);np.testing.assert_equal(before['features'],after['features']);np.testing.assert_equal(before['cells'],after['cells']);np.testing.assert_equal(model.predict(before['features'].astype(np.float32)),model.predict(after['features'].astype(np.float32)))
 results.append(dict(case=case,full_exposures_exact=counts[0],truncated_exposures_exact=counts[1],future_payload_rejected_before_read=True,model_scores_same_prefix=True,original_query_times_verified=True))
(H/'learned_memory_causality_checks.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),results=results,GT_labels_not_read=True,not_online_interface_validation=True),indent=2)+'\n');print(results)
