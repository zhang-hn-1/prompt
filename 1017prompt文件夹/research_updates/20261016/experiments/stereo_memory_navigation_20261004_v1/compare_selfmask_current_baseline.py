from pathlib import Path
import json,numpy as np,datetime
H=Path(__file__).resolve().parent;A=H/'prior_rgb_current_run';B=H/'selfmask_rgb_current_run';a=np.load(A/'physics.npz');b=np.load(B/'physics.npz');fields={k:bool(k in b.files and np.array_equal(a[k],b[k],equal_nan=True)) for k in a.files if k not in ('raw_simulation_time_s','raw_physics_step_index')};exactRGB=0;rgbMAE=[];samepred=0;total=0
for f in sorted((B/'source_predictions').glob('*.npz')):
 x=np.load(f);old=A/'source_predictions'/f.name
 if not old.exists():continue
 y=np.load(old);total+=1;samepred+=int(all(np.array_equal(x[k],y[k],equal_nan=True) for k in ('depth_m','confidence','disparity_px')))
 newrgb=np.load(B/'online_inputs'/f.name);oldrgb=np.load(A/'online_inputs'/f.name)
 for eye in ('left_rgb','right_rgb'):
  exactRGB+=int(np.array_equal(newrgb[eye],oldrgb[eye]));rgbMAE.append(float(np.abs(newrgb[eye].astype(float)-oldrgb[eye]).mean()))
ra=json.loads((A/'policy_records.json').read_text());rb=json.loads((B/'policy_records.json').read_text());sh=json.loads((B/'shadow_records.json').read_text());n=min(len(ra),len(rb));requests=np.array([r['local']['request'] for r in ra[:n]])!=np.array([r['local']['request'] for r in rb[:n]])
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),baseline='prior_rgb_current_run',candidate='selfmask_rgb_current_run',same_length=len(ra)==len(rb),same_full_nonraw_physics=bool(all(fields.values())),physical_fields_exact=fields,request_blocks_different=int(requests.any(1).sum()),same_full_execution_states=json.loads((A/'execution_states.json').read_text())==json.loads((B/'execution_states.json').read_text()),RGB_pair_inference_frames=total,RGB_eyes_byte_equal=exactRGB,RGB_eye_MAE_max=max(rgbMAE),source_predictions_exact_frames=samepred,candidate_vs_same_stream_shadow_raw_requests_different=sum(x['local']['request']!=y['request'] for x,y in zip(rb,sh)),scope='offline actual baseline/candidate identity check; shadow commands unexecuted',whole_complete=False)
(H/'selfmask_rgb_current_baseline_comparison.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
