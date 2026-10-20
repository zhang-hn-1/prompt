"""Actual source-only inputs plus future-clock/truncated-history negative tests; no NN/GT."""
from pathlib import Path
import json,hashlib,datetime,numpy as np
from memory_geometry_prompt_v2 import memory_geometry_prompt,world_up_pair
H=Path(__file__).resolve().parent;I=H/'any2full_memory_inputs_v1';D=H/'qualified_hazard_navigation_run_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();s=json.loads((I/'input_seal.json').read_text());assert all(sha(p)==v for p,v in s['input_sha256'].items());out=H/'any2full_sealed_input_causality_v2.json';assert not out.exists();rows=[];count=0
class FuturePayload:
 def __array__(self,*a,**k):raise AssertionError('future payload consumed')
for r in json.loads((I/'records.json').read_text()):
 frame=r['frame'];eye=r['eye'];name=f'frame_{frame:03d}.npz';proof={k:v for k,v in np.load(D/'appearance_predictions'/name).items()};pose={k:v for k,v in np.load(D/'known_poses'/name).items()};mask={k:v for k,v in np.load(D/'masks'/name).items()};rgb=np.load(D/'online_inputs'/name);sealed=np.load(I/f'frame_{frame:03d}_{eye}.npz');built=memory_geometry_prompt(proof,pose,mask,r['current_time_s'],frame,eye);image,dep=world_up_pair(rgb[eye+'_rgb'],built['prompt_depth_m']);assert np.array_equal(image,sealed['current_rgb_world_up']) and np.array_equal(dep,sealed['prompt_depth_world_up_m']);assert np.array_equal(built['past_point_index'],sealed['past_point_index_native'])
 for layer,field in [('proof','current_appearance_time_s'),('proof','source_geometry_time_s'),('pose','time_s'),('mask','time_s')]:
  a=dict(proof);b=dict(pose);c=dict(mask);layerdict={'proof':a,'pose':b,'mask':c}[layer];layerdict[field]=r['current_time_s']+.1;a['past_world']=FuturePayload();b[eye+'_camera_usd_world']=FuturePayload();c[eye]=FuturePayload()
  try:memory_geometry_prompt(a,b,c,r['current_time_s'],frame,eye)
  except ValueError:count+=1
  else:raise AssertionError('future clock accepted')
 # A truncated history has no historical points; helper must not synthesize them.
 a=dict(proof)
 for k in ['past_world','past_query_id','past_partner_id','left_uv','right_uv','left_z','right_z','patch_NCC','patch_std']:a[k]=np.asarray(a[k])[:0]
 truncated=memory_geometry_prompt(a,pose,mask,r['current_time_s'],frame,eye);assert truncated['nonzero_prompts']==0
 rows.append(dict(frame=frame,eye=eye,sealed_RGB_prompt_exact=True,negative_future_clock_tests=4,truncated_history_prompts=0,source_birth_time=float(built['source_geometry_time_s'])))
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_sha256=sha(Path(__file__)),input_seal_sha256=sha(I/'input_seal.json'),rows=rows,future_guard_tests=count,GT_read=False,learned_forward_calls=0,NOT_online_shadow_or_NAV_validation=True)
out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
