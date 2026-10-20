from pathlib import Path
import json,hashlib,numpy as np
H=Path(__file__).resolve().parent
names=['sgbm_baseline35_e0_run_v1','sgbm_baseline35_e1_coverage_run_v1','sgbm_baseline35_e2_run_v1','sgbm_baseline35_native_e0_run_v1','sgbm_baseline35_native_e1_run_v1','sgbm_baseline35_native_e2_run_v1']
if __name__=='__main__':
 rows=[]
 for name in names:
  d=H/name;maps=sorted((d/'maps').glob('*.npz'));assert maps,name
  alphabet=set();codes=set();observed_free=0;negative_unknown=0;priors=0;identity={}
  for p in maps:
   a=np.load(p);lab=a['labels'];alphabet.update(np.unique(lab).tolist());codes.update(np.unique(a['codes']).tolist());observed_free+=int((lab==0).sum());negative_unknown+=int((lab==-1).sum());priors+=int((a['ground_prior']|a['safe_start_prior']).sum());identity[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
  rows.append(dict(sequence=name,map_captures=len(maps),label_alphabet=sorted(alphabet),code_alphabet=sorted(codes),explicit_observed_free_cell_snapshots=observed_free,unknown_cell_snapshots=negative_unknown,prior_cell_snapshots=priors,map_sha256=identity))
 result=dict(question_sha256=hashlib.sha256((H/'memory_reobservation_dataset_question_v1.json').read_bytes()).hexdigest(),sequences=rows,all_have_explicit_negative_truth=all(r['explicit_observed_free_cell_snapshots']>0 for r in rows),classification='map_outputs_are_not_independent_reobservation_ground_truth',new_physics=0,new_fit=0,old_models_unchanged=True,whole_mission_complete=False)
 p=H/'memory_reobservation_map_availability_v1.json';assert not p.exists();p.write_text(json.dumps(result,indent=2)+'\n');print([{k:v for k,v in r.items() if k!='map_sha256'} for r in rows])
