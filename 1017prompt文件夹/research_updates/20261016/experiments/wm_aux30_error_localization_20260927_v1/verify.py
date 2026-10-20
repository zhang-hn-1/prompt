from pathlib import Path
import json,hashlib,time,math,re,subprocess
S=Path(__file__).resolve().parent;read=lambda f:json.loads(Path(f).read_text());p=read(S/'protocol.json');d=read(S/'results.json');rows=read(S/'rows.json');ROOT=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark');R=ROOT/'research/ball_wm/runs'/S.name
sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
assert d['status']=='complete' and d['self_checks']==7 and d['raw_sequence_alignment_checks']==16
assert sha(S/'analyze.py')==p['script_sha256']
for f,h in read(S/'input_hashes.json').items():assert sha(f)==h,f
for f,h in p['source_sha256'].items():assert sha(ROOT/f)==h,f
assert len(rows)==3696 and len({(r['episode'],r['start_frame']) for r in rows})==3696
for split,expected in [('old_fit',376),('new_fit',188),('val',94),('test',94)]:
 rs=[r for r in rows if r['split']==split and r['primary']];assert len(rs)==expected
 for h in ['0.1','0.3','0.5']:
  for model in ['A','B','CV']:
   for key in ['xy_cm','yaw_deg','xy_mse','yaw_mse']:
    mean=sum(r['h'][h][model][key] for r in rs)/len(rs);assert abs(mean-d['summary'][split]['all']['horizons'][h][model][key])<1e-12
 for key,groups in d['summary'][split]['groups'].items():assert sum(v['count'] for v in groups.values())==expected
assert len(read(S/'event_rows.json'))==105
for split,n in [('val',11),('test',10)]:assert all(x['count']==n for x in d['event_aligned'][split].values())
x=read(S/'training_exposure.json');assert x['arms']['A_original']['at_selected_checkpoint']['joint']==2715 and x['arms']['B_expanded']['at_selected_checkpoint']['joint']==3141
for model in ['A','B']:
 for z in d['gradient_diagnostic'][model].values():assert -1<=z['shared_xy_yaw_cosine']<=1 and 0<z['yaw_fraction_of_loss']<1
base=ROOT/'research/ball_wm/runs/wm_aux30_data_expansion_20260927_v1';old=read(base/'protocol.json');assert all(not Path(j['output']).exists() for j in read(old['fresh_manifest'])['jobs'])
assert read(base/'B_expanded/assessment.json')['passed']==24
assert '16 passed' in (S/'behavior_tests.log').read_text()
report=S/'world_model_aux30_error_localization_20260927.md'
for link in re.findall(r'\]\((/[^)]+)\)',report.read_text()):assert Path(link).exists() or (S/Path(link).name).exists() or Path(link).name=='final_verification.json',link
check=subprocess.run(['git','diff','--check'],cwd=ROOT,capture_output=True,text=True);assert check.returncode==0
result={'passed':True,'verified_epoch':time.time(),'cached_windows':3696,'primary_historical_windows':188,'aligned_event_windows':105,'frozen_input_files':len(read(S/'input_hashes.json')),'frozen_sources':len(p['source_sha256']),'all_inputs_and_sources_unchanged':True,'self_checks':7,'behavior_tests_passed':16,'prior_model_gate':24,'new_training_runs':0,'new_simulations':0,'fresh_untouched':True,'report_sha256':sha(report),'files_sha256':{f.name:sha(f) for f in S.iterdir() if f.is_file() and f.name!='final_verification.json'}}
(S/'final_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='files_sha256'},indent=2))
