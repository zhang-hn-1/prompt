"""Read held labels ONLY after all predictions sealed; frozen model gates."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
from cell_proxy_metrics import confusion
H=Path(__file__).resolve().parent;O=H/'learned_memory_held_assessment';O.mkdir(exist_ok=False);P=H/'learned_memory_held_predictions';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat();seal=json.loads((P/'all_predictions_pretruth_sha256.json').read_text())
for p,s in seal['sha256'].items():assert sha(p)==s,p
cases=['memory_validation_far_run','memory_validation_near_run'];paths=[H/case/'offline_occupancy_label.npz' for case in cases];(O/'protocol.json').write_text(json.dumps(dict(utc=now(),prediction_seal_sha256=sha(P/'all_predictions_pretruth_sha256.json'),prediction_seal_utc=seal['utc'],label_sha256={str(p):sha(p) for p in paths},assessment_source_sha256=sha(Path(__file__).resolve()),proxy_only=True,frozen_threshold=.5),indent=2)+'\n');results=[]
for case in cases:
 occ=np.load(H/case/'offline_occupancy_label.npz');pred=[];base=[];truth=[];frames=[]
 for p in sorted((P/case).glob('*.npz')):
  d=np.load(p);xy=d['origin']+d['cells']*float(d['resolution']);ij=np.rint((xy-occ['origin'])/float(occ['resolution'])).astype(int);valid=(ij>=0).all(1)&(ij<occ['labels'].shape).all(1);y=occ['labels'][ij[valid,0],ij[valid,1]]>0;a=d['scores'][valid]>=.5;b=d['baseline_history_labels'][valid]==1;pred.append(a);base.append(b);truth.append(y);frames.append(dict(frame=p.stem,time_s=float(d['time_s']),model=confusion(a,y),baseline=confusion(b,y),GT_total_occupied_cells=int((occ['labels']>0).sum()),eligible_occupied_cells=int(y.sum()),GT_occupied_cells_outside_eligible_set=int((occ['labels']>0).sum()-y.sum())))
 a=confusion(np.concatenate(pred),np.concatenate(truth));b=confusion(np.concatenate(base),np.concatenate(truth));positive=a['TP']+a['FN'];checks=dict(false_positive_no_worse=a['FP']<=b['FP'],false_negative_no_worse=a['FN']<=b['FN'])
 if 'far' in case:checks.update(precision=a['precision'] is not None and a['precision']>=.95,recall=a['recall'] is not None and a['recall']>=.80,at_least_25_true_positive=a['TP']>=25)
 elif positive:checks.update(precision=a['precision'] is not None and a['precision']>=.95,recall=a['recall'] is not None and a['recall']>=.80)
 result=dict(case=case,model=a,baseline=b,checks=checks,layer_pass=all(checks.values()),eligible_only=True,actual_obstacle_coverage_passed=False,recall_coverage_NA_if_zero_eligible_positive=positive==0,frames=frames);results.append(result)
accepted=all(r['layer_pass'] for r in results);report=dict(utc=now(),sequences=results,model_validation_passed=accepted,online_shadow_triggered=False,navigation_passed=False,source_depth_admission_unchanged_rejected=True,whole_research_complete=False,decision='accept_for_one_readonly_shadow_only' if accepted else 'reject_single_candidate_no_more_fit_or_sequences');(O/'assessment.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='sequences'},indent=2));print(json.dumps([{k:v for k,v in r.items() if k!='frames'} for r in results],indent=2))
