from pathlib import Path
import hashlib,json,datetime,ast
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
parent=H/'bounded_prefix_e1_protocol_v3.json';p=json.loads(parent.read_text())
for group in ('source_sha256','input_sha256'):
 for f,s in p[group].items():assert sha(f)==s,f
for name in ('forward_course_geometry_v1.py','forward_course_geometry_adapter_v4.py','forward_course_score_evidence_v1.py','assess_forward_course_geometry_e1_v1.py','run_forward_course_geometry_e1_v2.py','run_forward_course_geometry_e1_stage_v2.py','test_forward_course_geometry_real_factory_v4.py','test_forward_course_composition_snapshot_v3.py','test_forward_course_score_evidence_v1.py','freeze_forward_course_geometry_e1_v1.py'):
 f=H/name;ast.parse(f.read_text());p['source_sha256'][str(f)]=sha(f)
for name in ('forward_course_geometry_consumer_design_v1.json','forward_course_geometry_navigation_scope_v1.json','forward_course_prelaunch_cpu_verification_v1.txt'):
 f=H/name;p['input_sha256'][str(f)]=sha(f)
p.update(stage='qualified current forward course geometric consumer original35 known-E1 two-arm',only_change='geometric_curvature_argument_before_original_composition',design=str(H/'forward_course_geometry_navigation_scope_v1.json'),parent_protocol_sha256=sha(parent),utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),launch_admission='PENDING independent full envelope review; no physical process launched',classification_scope='Qualified event may be protocol-equivalent; no observed isolated first actual fork plus exact prefix => unassessed_event_or_interface, never performance rejection')
p.pop('parent_envelope_v1_sha256',None);p.pop('parent_envelope_v2_sha256',None)
p['numeric_gates'].update(actual_intervention=True,first_request_fork_at_intervention='first actual fork must consume qualified course; earlier equivalent events permitted')
out=H/'forward_course_geometry_e1_protocol_v1.json';assert not out.exists();out.write_text(json.dumps(p,indent=2)+'\n')
print('PENDING review',len(p['source_sha256']),len(p['input_sha256']),sha(out))
