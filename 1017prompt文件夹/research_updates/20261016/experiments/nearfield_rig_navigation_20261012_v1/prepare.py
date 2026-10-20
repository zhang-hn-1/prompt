from pathlib import Path
import json,hashlib
D=Path(__file__).resolve().parent;W=D.parents[1];old=W/'experiments/visual_imu_navigation90_20261012_v1'
selected=json.loads((D/'geometry_assessment.json').read_text())['selected'];assert selected
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for arm,scene in [('baseline','e2'),('candidate','e2'),('candidate','e0'),('candidate','e1')]:
 key=arm+'_'+scene;src=(old/(scene+'_driver.py')).read_text()
 src=src.replace("D/(", "D/(") # no semantic transformation here
 src=src.replace("D/'"+scene+"_protocol.json'", "D/'"+key+"_protocol.json'").replace("out=D/'"+scene+"_run'","out=D/'"+key+"_run'")
 src=src.replace("a.name='visual_imu_navigation90_"+scene+"'","a.name='nearfield_rig_"+key+"'")
 config=json.loads((old/(scene+'_protocol.json')).read_text())
 if arm=='candidate':
  hook='''\nfrom rig_adapter import Rig,RigMask,RigStereo,native_pose,paired,validate,install_runtime_geometry\nRIG=Rig(**%r)\ninstall_runtime_geometry(RIG)\ncurrent_native_pose=lambda record,t:native_pose(record,t,RIG)\nSGBMStereo=lambda:RigStereo(RIG)\nRobotSelfMask=lambda:RigMask(RIG)\npaired_sources=paired\nvalidate_capture=lambda row:validate(row,RIG)\ncamera_rig_metadata=RIG.metadata\n''' % selected
  src=src.replace('from rig_inputs import validate_capture','from rig_inputs import validate_capture\n'+hook)
  src=src.replace("only_source_change='none_SGBM_frozen'","only_source_change='rig_geometry_and_dependent_calibration_masks_only_SGBM_parameters_frozen'")
  src=src.replace("comparison='fixed_classical_baseline_remaining_conditions_not_accepted_improvement_historical_same_episode_no_fullcausal_RGB_AB'","comparison='same_source_new_baseline_vs_forward_rig_development_E2_RGB_diff_expected'")
  config['rig_extrinsics']=str(D/(selected['name']+'_extrinsics.npz'));config['static_masks']=str(D/(selected['name']+'_masks.npz'))
  for p in [config['rig_extrinsics'],config['static_masks']]:config['input_sha256'][p]=sha(p)
 src=src.replace("role='ONE pose-source trial; registration and navigation current native IMU encoder; initial privileged gauge and ideal lowlevel IMU; not pure RGB'","role='nearfield rig comparison; native IMU encoder; initial gauge and lowlevel ideal IMU privileged; not pure RGB'")
 path=D/(key+'_driver.py');path.write_text(src)
 config['source_sha256'][str(path)]=sha(path)
 for p in ['rig_adapter.py','protocol.json','geometry_screen.py','geometry_assessment.json']:config['source_sha256'][str(D/p)]=sha(D/p)
 keep={'source_sha256','input_sha256','scene_root','rig_extrinsics','static_masks','pid_config','inner_config','episode_manifest','episode_id','occupancy'}
 config={**{k:v for k,v in config.items() if k in keep},'parent_metadata':{k:v for k,v in config.items() if k not in keep}}
 config.update(new_rig_arm=arm,new_rig_candidate=selected if arm=='candidate' else None,version='nearfield_rig_navigation_v1_'+key,sim_budget_s=90.,requests_max=900,whole_stage_maximum_actual_sequences=4,maximum_actual_sequences=1,no_retry=True,acceptance=json.loads((D/'protocol.json').read_text())['navigation_gate'],local_screen=json.loads((D/'protocol.json').read_text())['geometry_gate'],source_accuracy_gate=json.loads((D/'protocol.json').read_text())['online_depth_gate'],only_source_change='camera_geometry_and_dependent_calibration_masks' if arm=='candidate' else 'none',only_control_change='none',only_pose_change='camera_extrinsic_registration_only' if arm=='candidate' else 'none',comparison='new_same_source_E2_AB_development_comparison_RGB_may_differ',scope='visual+nativeIMUencoder;initial_gauge_flatground_safe_start_and_lowlevelidealIMU_privileged_not_pureRGB',future_truth_for_scoring_only=True)
 (D/(key+'_protocol.json')).write_text(json.dumps(config,indent=2)+'\n')
print('prepared four explicitly bounded arm drivers; no launch')
