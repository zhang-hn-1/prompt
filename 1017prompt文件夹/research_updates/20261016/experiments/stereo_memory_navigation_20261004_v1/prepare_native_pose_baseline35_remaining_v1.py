from pathlib import Path
import json,hashlib,ast,datetime
H=Path(__file__).resolve().parent
s=(H/'run_native_pose_baseline35_e0_v1.py').read_text().replace("choices=['e0']","choices=['e1','e2']").replace("a.name='native_pose_baseline35_e0'","a.name='native_pose_baseline35_'+a.condition+'_coverage'").replace("'native_pose_baseline35_e0_protocol_v1.json'","('native_pose_baseline35_'+a.condition+'_coverage_protocol_v1.json')").replace("'native_pose_baseline35_e0_run_v1'","('native_pose_baseline35_'+a.condition+'_coverage_run_v1')")
ast.parse(s);driver=H/'run_native_pose_baseline35_remaining_v1.py';driver.write_text(s)
for n in ('e1','e2'):
 c=json.loads((H/'native_pose_baseline35_e0_protocol_v1.json').read_text());old=json.loads((H/(f'sgbm_baseline35_{n}_coverage_protocol_v1.json' if n=='e1' else 'sgbm_baseline35_e2_protocol_v1.json')).read_text())
 for k in ('scene_root','episode_manifest','episode_id','occupancy','input_sha256'):c[k]=old[k]
 c['source_sha256'].pop(str(H/'run_native_pose_baseline35_e0_v1.py'));c['source_sha256'][str(driver)]=hashlib.sha256(driver.read_bytes()).hexdigest()
 c.update(task='remaining fixed traditional SGBM estimated-pose baseline coverage',role='baseline evaluation completion, not accepted improvement transfer',version='v1_native_pose_remaining_baseline_coverage',candidate_stage='same frozen native consumer source/controller, no new candidate',comparison=f'sgbm_baseline35_{n}_coverage_run_v1' if n=='e1' else 'sgbm_baseline35_e2_run_v1',scope='initial privileged gauge flatground safe-start TaskH not pureRGB originalstrict',maximum_actual_sequences=1,whole_stage_maximum_actual_sequences=2,predeclared_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),next_if_rejected='record failed baseline; no matcher/controller/pose rescue or threshold sweep')
 (H/f'native_pose_baseline35_{n}_coverage_protocol_v1.json').write_text(json.dumps(c,indent=2)+'\n')
print('remaining coverage prepared, not started; requires normalclose native E0 and interface evidence')
