"""Post-close interface/count/video verification only; never a source score."""
from pathlib import Path
import json,hashlib,numpy as np,cv2
H=Path(__file__).resolve().parent;D=H/'persistent_context_C1_run_v1';B=H/'paired_current_A2_run_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();s=json.loads((H/'persistent_context_online_status_v1.json').read_text());assert s['exit_code']==0;seal=json.loads((D/'artifact_manifest_post_close.json').read_text());assert all(sha(p)==v for p,v in seal['artifact_sha256'].items());p=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());physics=np.load(D/'physics.npz');c=json.loads((D/'camera_clocks.json').read_text());contacts=json.loads((D/'physics_contacts.json').read_text());birth=np.load(D/'immutable_possible_set_birth.npz');rows=[]
for f in sorted((D/'persistent_context').glob('context_*.npz')):
 z=np.load(f);assert bool(z['raw_birth_unchanged']) and bool(z['physical_state_unchanged']);assert float(z['birth_time_s'])==float(birth['time_s']);assert str(z['hypothesis_sha256'])==sha(D/'immutable_possible_set_birth.npz');rows.append(dict(frame=int(z['frame']),time_s=float(z['time_s']),left={k:int((z['left']==k).sum()) for k in np.unique(z['left'])},right={k:int((z['right']==k).sum()) for k in np.unique(z['right'])}))
assert rows and len(birth['raw_world_voxel_centres'])>0;assert len(p)==40 and len(states)==41 and len(c)==41 and len(physics['raw_command_target'])==201;assert all(np.all(np.asarray(v['local']['request'])==0) for v in p);assert all(not v['contact'] for v in contacts);assert len(list((D/'sensor_packets').glob('source_*.npz')))==9
videos={}
for name in ['stereo_view','third_person']:
 v=cv2.VideoCapture(str(D/(name+'.mp4')));n=0;fps=v.get(cv2.CAP_PROP_FPS)
 while True:
  ok,im=v.read()
  if not ok:break
  if n==20:cv2.imwrite(str(D/(name+'_frame020.png')),im)
  n+=1
 v.release();assert n==41 and abs(fps-10)<1e-8;videos[name]=dict(decoded_frames=n,fps=fps)
bp=np.load(B/'physics.npz');comparisons={k:bool(np.array_equal(physics[k],bp[k])) for k in physics.files if k in bp.files};bs=json.loads((B/'states.json').read_text());statecomparisons={k:bool(np.array_equal(np.asarray([v[k] for v in states]),np.asarray([v[k] for v in bs]))) for k in states[0]};bpol=json.loads((B/'policy_records.json').read_text());requesteq=all(np.array_equal(a['local']['request'],b['local']['request']) for a,b in zip(p,bpol)) and len(p)==len(bpol)
r=dict(scope='ACTUAL_ONLINE_READONLY_CONTEXT_INTERFACE_ONLY',requests=40,captures=41,physics_ticks=201,NN_updates=9,online_context_updates=len(rows),birth_frame=int(birth['frame']),birth_voxels=len(birth['raw_world_voxel_centres']),all_birth_and_physical_flags_unchanged=True,contacts=0,available_contact_samples=sum(bool(v['available']) for v in contacts),videos=videos,baseline_context_off_case='old A2 same4s zero prefix',baseline_requests_exact=requesteq,baseline_state_fields_exact=statecomparisons,baseline_physics_fields_exact=comparisons,rows=rows,no_source_scoring_or_model_or_nav_admission=True,known_initial_gauge_idealgyro_floor_prior_preserved=True,code_sha256=sha(__file__))
(D/'online_context_interface_verification_v1.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k not in ['rows','baseline_physics_fields_exact']},indent=2));print('physics exact',sum(comparisons.values()),'/',len(comparisons),'different',[k for k,v in comparisons.items() if not v])
