from pathlib import Path
import json,numpy as np
from learned_memory_risk_dataset_v1 import offline_teacher_labels,sha
from learned_memory_risk_identity_v1 import verify_prefit_identity
H=Path(__file__).resolve().parent
if __name__=='__main__':
 prefit_sha=verify_prefit_identity(H)
 design=json.loads((H/'learned_memory_risk_qualifier_design_v1.json').read_text());model=json.loads((H/'learned_memory_risk_model_v1.json').read_text());seal=json.loads((H/'learned_memory_risk_holdout_predictions_seal_v1.json').read_text());assert seal['two_complete_sequence_predictions_sealed_before_teacher'] and seal['model_sha256']==sha(H/'learned_memory_risk_model_v1.json')
 assert seal['prefit_identity_sha256']==prefit_sha
 for p,h in seal['outputs_sha256'].items():assert sha(p)==h,p
 results={};teacher_inputs={}
 for name in design['fresh_whole_sequence_holdout']:
  pred=np.load(H/(name+'_risk_predictions_v1.npz'));meta=json.loads((H/(name+'_risk_predictions_v1.json')).read_text());y,height,inputs=offline_teacher_labels(H/name,meta['index']);teacher_inputs.update(inputs);keep=pred['keep'];valid=y>=0;assert len(y)==len(keep)
  floor=y==0;hazard=y==1;f=int(floor.sum());h=int(hazard.sum());retainedfloor=int((floor&keep).sum());retainedhazard=int((hazard&keep).sum());reduction=1-retainedfloor/f if f else None;recall=retainedhazard/h if h else None;g=dict(label_coverage=f>=20 and h>=20,floor_false_risk_reduction=reduction is not None and reduction>=.3,hazard_retention=recall is not None and recall>=.9)
  results[name]=dict(points=len(y),floor=f,hazard=h,unlabelled=int((~valid).sum()),baseline_all_selected_risks=True,retained_floor=retainedfloor,retained_hazard=retainedhazard,false_floor_reduction=reduction,hazard_retention=recall,constant_drop_all=not bool(keep.any()),gates=g,passed=all(g.values()))
 accepted=model['converged'] and all(r['passed'] and not r['constant_drop_all'] for r in results.values());insufficient=any(not r['gates']['label_coverage'] for r in results.values());r=dict(results=results,classification='unassessed_label_coverage' if insufficient else 'offline_learning_memory_accept' if accepted else 'learning_memory_candidate_reject',candidate_accepted=accepted,model_converged=model['converged'],teacher_inputs_sha256=teacher_inputs,predictions_sealed_before_teacher=True,whole_sequence_split=True,same_scenes_no_spatial_OOD_claim=True,observed_points_only_not_unseen_hazards=True,not_navigation_or_runtime_admission=True,not_dynamics_world_model=True,whole_mission_complete=False)
 out=H/'learned_memory_risk_holdout_assessment_v1.json';assert not out.exists();out.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='teacher_inputs_sha256'}))
