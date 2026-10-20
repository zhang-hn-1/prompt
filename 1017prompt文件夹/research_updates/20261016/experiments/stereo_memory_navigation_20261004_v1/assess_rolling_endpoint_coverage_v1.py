from pathlib import Path
import sys,json,hashlib
H=Path(__file__).resolve().parent
if __name__=='__main__':
 layer,scene=sys.argv[1:];assert layer in ('known','native') and scene in ('e1','e2')
 import importlib
 module=importlib.import_module('rolling_endpoint_coverage_'+layer+'_assessment_v1');module.CASE='rolling_endpoint_coverage_'+layer+'_'+scene
 p=H/(module.CASE+'_assessment_v1.json');assert not p.exists()
 cfg=json.loads((H/'rolling_endpoint_coverage_protocol_v1.json').read_text());module.verify(cfg['source_sha256']);module.verify(cfg['input_sha256'])
 try:
  metric,raw=module.arm('paced',cfg)
  events=[r['frame'] for r in raw['rows'] if r.get('rolling_control_applied')]
  result=dict(layer=layer,condition=scene,metrics=metric,classification='conditional_task_arrival' if metric['success'] and metric['raw_contacts_zero'] else 'measured_navigation_failure',actual_new_qualification_changes=len(events),first_change=events[0] if events else None,causal_navigation_improvement=False,historical_comparison_descriptive_only=True,not_strict_visual=True,whole_mission_complete=False)
 except (AssertionError,ValueError,KeyError,OSError) as e:
  result=dict(layer=layer,condition=scene,classification='unassessed_evidence_or_execution',reason=repr(e),scientific_rejection=False,whole_mission_complete=False)
 p.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
