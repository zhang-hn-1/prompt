from pathlib import Path
import json,sys
import numpy as np,torch
S=Path(__file__).resolve().parent;ROOT=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark');RUNS=ROOT/'research/ball_wm/runs';AB=RUNS/'wm_aux30_data_expansion_20260927_v1';sys.path.insert(0,str(AB));from training_scope import batch_schedule,expanded_partition
read=lambda f:json.loads(Path(f).read_text());old=torch.load(RUNS/'wm_aux30_acceleration_20260924_v1/feature_windows.pt',map_location='cpu',weights_only=False);new=torch.load(RUNS/'wm_aux30_independent_calibration_20260926_v1/calibration_features.pt',map_location='cpu',weights_only=False);meta=old['metadata']+new['metadata'];lookup={(r['episode'],r['start_frame']):r for r in read(S/'rows.json')};parts=expanded_partition(old['metadata'],old['partitions'],new['metadata'])
masks={};joint={}
for split in ['old_fit','new_fit','val','test']:
 ids=old['partitions'][['old_fit','val','test'].index(split)] if split!='new_fit' else list(range(len(old['metadata']),len(meta)))
 rs=[lookup[(meta[i]['episode'],meta[i]['start_frame'])] for i in ids];selected=[r for r in rs if r['groups']['motion']=='reverse' and r['groups']['speed']=='fast' and r['groups']['aux']=='large'];joint[split]={'total':len(rs),'reverse_fast_large':len(selected),'positive_aux':sum(r['actual_aux_deg']>=15 for r in selected),'negative_aux':sum(r['actual_aux_deg']<=-15 for r in selected)}
for key in ['joint','positive_joint','negative_joint']:
 masks[key]=np.array([int((r:=lookup[(m['episode'],m['start_frame'])])['groups']['motion']=='reverse' and r['groups']['speed']=='fast' and r['groups']['aux']=='large' and (key=='joint' or (r['actual_aux_deg']>=15 if key=='positive_joint' else r['actual_aux_deg']<=-15))) for m in meta])
result={'descriptive_joint_group':'Post-hoc intersection of prespecified reverse/fast/large-aux groups; not new acceptance threshold.','all_window_coverage':joint,'arms':{}}
for arm,ids,epochs in [('A_original',old['partitions'][0],30),('B_expanded',parts[0],20)]:
 at=read(AB/arm/'selection.json')['selected']['optimizer_steps'];count={k:0 for k in masks};snap=None
 for step,ix in enumerate(batch_schedule(ids,epochs,2,3072),1):
  for k,mask in masks.items():count[k]+=int(mask[ix].sum())
  if step==at:snap=dict(count)
 result['arms'][arm]={'selected_updates':at,'selected_total_window_presentations':at*2,'at_selected_checkpoint':snap,'whole_budget':count}
(S/'training_exposure.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
