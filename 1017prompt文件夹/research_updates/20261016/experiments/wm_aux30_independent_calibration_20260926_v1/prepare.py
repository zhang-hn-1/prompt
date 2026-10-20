from pathlib import Path
import json,hashlib,math,time,shutil
R=Path(__file__).resolve().parent;ROOT=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark');OUT=ROOT/'research/ball_wm/runs'/R.name;DATA=ROOT/'research/ball_wm/data/wm_aux30_calibration_20260926_v1'
G=ROOT/'research/ball_wm/runs/wm_aux30_confidence_gate_20260926_v1';A=ROOT/'research/ball_wm/runs/wm_aux30_acceleration_20260924_v1';F=ROOT/'research/ball_wm/runs/wm_aux30_residual_20260923_v1/fresh_sequence_manifest.json'
read=lambda p:json.loads(Path(p).read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
programs=[
 ('easy_1_cal_forward_exchange',1,47,[(4,.65,0),(4,.65,22),(4,.35,-22),(4,0,0),(4,-.65,-15),(4,0,0)]),
 ('easy_1_cal_reverse_exchange',1,137,[(4,-.75,0),(4,-.35,25),(4,-.35,-25),(4,0,0),(4,.60,10),(4,0,0)]),
 ('easy_2_cal_speed_histories',2,227,[(3,.25,16),(5,.85,16),(4,0,0),(3,-.25,-16),(5,-.85,-16),(4,0,0)]),
 ('easy_2_cal_low_and_reversal',2,317,[(4,.20,-12),(4,0,-12),(4,-.20,12),(4,.60,24),(4,-.60,-24),(4,0,0)])]
fresh=read(F);old=read(ROOT/'research/ball_wm/runs/wm_aux30_diverse_20260922_v1/collection_manifest.json');jobs=[]
oldarrays=[]
for j in old['jobs']+fresh['jobs']:
 seq=read(j['command_sequence']);arr=[]
 for x in seq['segments']:arr.extend([x['request']]*x['steps'])
 oldarrays.append(arr)
for n,(name,scene_id,yaw,segments) in enumerate(programs):
 seq={'schema':'wm_diverse_requested_sequence_v1','id':name,'physics_hz':50,'stereo_hz':10,'total_steps':1200,'segments':[]}
 for i,(sec,main,aux) in enumerate(segments):seq['segments'].append({'name':f'segment_{i+1:02d}','steps':sec*50,'seconds':sec,'request':[main,math.radians(aux)],'request_main_rad_s':main,'request_aux_deg':aux})
 assert sum(x['steps'] for x in seq['segments'])==1200
 arr=[]
 for x in seq['segments']:
  assert x['steps']%5==0 and abs(x['request_main_rad_s'])<=1 and abs(x['request_aux_deg'])<=30
  arr.extend([x['request']]*x['steps'])
 assert arr not in oldarrays
 file=R/f'{name}.json';file.write_text(json.dumps(seq,indent=2)+'\n')
 scene=f'/home/an/project/NavDP/assets/scenes/cluttered_easy/easy_{scene_id}/cluttered-{scene_id}.usd';output=DATA/f'easy_{scene_id}'/name;seed=2026092601+n
 command=['/home/an/miniconda3/envs/ball_wm/bin/python','research/ball_wm/scripts/collect_data_dubins.py','--scene',scene,'--num_episodes','1','--episode_len','1200','--seed',str(seed),'--initial_yaw_deg',str(yaw),'--command_sequence',str(OUT/file.name),'--main_accel_rad_s2','.5','--aux_rate_deg_s','30','--auxiliary_velocity_limit_rad_s','.30','--no_save_depth','--no_save_elevation','--save_preview','--audit_after_episode','--save_dir',str(output)]
 jobs.append({'id':name,'split':'calibration_only','scene':scene,'output':str(output),'seed':seed,'initial_yaw_deg':yaw,'command_sequence':str(OUT/file.name),'command_sequence_sha256':sha(file),'command':command,'expected_physics_ticks':1200,'expected_stereo_frames':240})
manifest={'frozen_before_collection_epoch':time.time(),'max_calibration_sequences':4,'seconds_per_sequence':24,'purpose':'New complete sequences in known base-training scenes, used only to calibrate frozen-model residual scaling; not fresh test','no_outcome_dependent_extra_sequences':True,'no_collision_or_failed_motion_filtering':True,'data_root':str(DATA),'jobs':jobs}
(R/'collection_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
shutil.copy2(G/'confidence_gate.py',R/'confidence_gate.py');shutil.copy2(G/'test_confidence_gate.py',R/'test_confidence_gate.py')
p=read(G/'protocol.json');inputs={}
for base in [G,A]:
 for file in base.iterdir():
  if file.is_file() and file.suffix in ['.json','.pt','.py','.md','.log','.png']:inputs[str(file)]=sha(file)
for j in fresh['jobs']:inputs[j['command_sequence']]=sha(j['command_sequence']);assert not Path(j['output']).exists()
inputs[str(F)]=sha(F)
protocol={'created_epoch':time.time(),'root':str(ROOT),'output':str(OUT),'data_root':str(DATA),'parent_gate_run':str(G),'frozen_acceleration_run':str(A),'frozen_rate_comparator':p['frozen_rate_comparator'],'candidate_count':1,'changed_variable':'Calibrate identical gate on four new sequences disjoint from base fit, replacing old training-set calibration; size192s->96s and state distribution also differ, not pure isolation-only ablation.','fit_only':'924 windows from four new24s sequences; no original base-fit, historical val/test, or reserved fresh used for fit/normalization. No random frame split.','architecture_and_optimization':{k:p[k] for k in ['parameters','features','coefficient','normalization','optimizer','deployment','fit_deployment_difference']},'historical_gate':p['historical_gate'],'acceleration_acceptance':p['acceleration_acceptance'],'extra_acceptance':p['extra_acceptance'],'historical_check_templates':[{k:v for k,v in x.items() if k not in ['ratio','delta','count','passed']} for x in read(G/'assessment.json')['checks']],'fresh_manifest':str(F),'fresh_manifest_sha256':sha(F),'fresh_condition':'Only after all28 original historical checks pass, collect and evaluate four reserved full sequences once, no fitting/checkpoint selection. Otherwise do not touch fresh outputs.','fresh_thresholds':'Same numerical thresholds: aggregate .5XY/CV<=.9, .5yaw/CV<=1; eachsequence .5XY/CV<=1; requestchange count>=10, .1/.5XY/CV<=1; aggregate .1XY/yawCV<=1; .5XY/yaw versus each rate and acceleration<=1.05; prefix tolerance1e-5.','no_navigation':True,'input_sha256':inputs,'source_sha256':p['source_sha256'],'manifest_sha256':sha(R/'collection_manifest.json'),'gate_source_sha256':sha(R/'confidence_gate.py')}
(R/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n');print(json.dumps({'jobs':[j['id'] for j in jobs],'manifest_sha256':protocol['manifest_sha256'],'same_gate_code':sha(R/'confidence_gate.py')==sha(G/'confidence_gate.py'),'fresh_untouched':True},indent=2))
