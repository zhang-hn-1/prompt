from pathlib import Path
import sys
H=Path('/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1'); E=H/'qualified_hazard_independent_evidence_v1';sys.path.insert(0,str(H))
for name,out in [('verify_qualified_hazard_source_v1.py','source_replay.json'),('verify_qualified_hazard_execution_v1.py','execution_replay.json'),('assess_qualified_hazard_geometry_v1.py','geometry_reassessment.json'),('compare_qualified_hazard_navigation_v1.py','comparison_reassessment.json'),('audit_qualified_hazard_failure_v1.py','failure_reassessment.json')]:
 code=(H/name).read_text()
 expected={'verify_qualified_hazard_source_v1.py':'qualified_hazard_source_verification_v1.json','verify_qualified_hazard_execution_v1.py':'qualified_hazard_execution_verification_v1.json','assess_qualified_hazard_geometry_v1.py':'qualified_hazard_geometry_assessment_v1.json','compare_qualified_hazard_navigation_v1.py':'qualified_hazard_navigation_comparison_v1.json','audit_qualified_hazard_failure_v1.py':'qualified_hazard_failure_audit_v1.json'}[name]
 code=code.replace("f=H/'"+expected+"'","f=Path("+repr(str(E/out))+")")
 exec(compile(code,str(H/name),'exec'),{'__file__':str(H/name),'__name__':'__main__'})
