"""First-fork point/raster representation comparison, frozen margin; no action API."""
from pathlib import Path
import json,numpy as np
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';O=H/'E1_guard_cell_trace_v1';row=json.loads((D/'policy_records.json').read_text())[331];guard=row['reference_decision']['guard'];points=np.vstack([np.asarray(guard[k]) for k in ['bridge_xy','suffix_xy']]);record=json.loads((O/'trace_pretruth.json').read_text())['datasets']['old'][0]['records'][0];pred=np.asarray(record['world'])[:2];truth=np.asarray(json.loads((O/'postseal_point_labels_v1.json').read_text())['rows'][0]['points'][0]['GT_world'])[:2]
def distance_polyline(p):
 distances=[]
 for key in ['bridge_xy','suffix_xy']:
  line=np.asarray(guard[key]);
  for a,b in zip(line[:-1],line[1:]):
   d=b-a;u=np.clip(np.dot(p-a,d)/np.dot(d,d),0,1);distances.append(float(np.linalg.norm(p-(a+u*d))))
 return min(distances)
margin=row['reference_decision']['tube_viability_screen']['existing_planning_clearance_m'];v={'scope':'ONE_FROZEN_POINT_VS_GRID_REPRESENTATION_DIAGNOSTIC_NOT_CONTROL_OR_SAFETY','frame':331,'single_risk_point_prediction':pred.tolist(),'postseal_truth_only':truth.tolist(),'predicted_point_distance_to_exact_bridge_suffix_m':distance_polyline(pred),'teacher_point_distance_to_exact_bridge_suffix_m':distance_polyline(truth),'recorded_continuous_cell_guard_m':guard['suffix_clearance_m'],'recorded_raster_tube_m':row['reference_decision']['tube_viability_screen']['minimum_raster_clearance_m'],'existing_planning_margin_unchanged_m':margin,'not_false_correspondence':True,'not_TTL_or_persistent_confirmation':True,'never_drop_observed_point':True,'quantization_boundary_distance_predicted_Y_m':abs(pred[1]-.950000005960466),'new_control_candidate_NOTRUN':True,'all_other_risk_points_not_checked_here':True};(O/'point_raster_clearance_comparison_v1.json').write_text(json.dumps(v,indent=2)+'\n');print(json.dumps(v,indent=2))
