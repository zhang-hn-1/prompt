from paths import *
import json,hashlib,ast,datetime,sys
for p in D.glob('*.py'):ast.parse(p.read_text())
import navigation
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
B=W/'experiments/four_camera_weak_baseline_20261017_v1';base=json.loads((B/'navigation_protocol.json').read_text());cfg=json.loads((D/'protocol.json').read_text())
files={str(p.resolve()):sha(p) for p in D.glob('*.py')}
for m in list(sys.modules.values()):
 f=getattr(m,'__file__',None)
 if f and Path(f).is_file() and (str(f).startswith(str(W)) or str(f).startswith(str(P))):files[str(Path(f).resolve())]=sha(f)
inputs=[D/'protocol.json',Path(cfg['episode_manifest']),Path(cfg['occupancy']),navigation.DEFAULT_CONTRACT.resolve(),navigation.DEFAULT_DRIVE_PROFILE.resolve(),W/'experiments/oracle_feasible_route_20260929_v9/gated_speed_012/selected_candidate.json',B/'e1_result.json',B/'navigation_protocol.json',B/'artifact_manifest_post_close.json',D/'e0_result.json',D/'interface_result.json']
base.update(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),name='current_four_camera_A_current_exposure90s',source_sha256=files,input_sha256={str(p):sha(p) for p in inputs},baseline_protocol_sha256=sha(D/'protocol.json'),physics_cap=1,recovery_cap=1,memory='A original CurrentExposureRiskGrid, most recent actual capture only. Source held until new exposure; absence never free.',P2_comparison_not_claimed=False,P2_partial_development_A_vs_known_B=True,C='Offline REJECT remains closed; no C physics; no complete3arm claim',comparison_numeric_gates=dict(arrival='distance<=.2m before90s and contact0',support='>=11nonempty exposures and>=100riskpoints required for visual memory interpretation',A_noninferiority='collision<=B0, source-supported blindmissing<=B+.05, fullchainP90<=B*1.1; descriptive development gates, not old source admission',false_memory_blocking_tradeoff='A netprogress>=B+.5m AND A false-map-cellseconds<=B*.8 reports benefit only if collision/blindmissing noninferiority also passes; no threshold rescue. B outcomes known before prospectiveA; not heldout/statistical causal efficacy.'),no_B_rerun=True,source_physical_cumulative='Current fixed weaksource physical B1done plus A1prospective=2; no old cap reopened; no additional resource grant implied',next_after_run='seal and evaluate same schema; compare actualA/B with trajectory/source divergence. Full3armC remains unqualified, no planner/learning increment claim.')
(D/'navigation_protocol.json').write_text(json.dumps(base,indent=2)+'\n');print('FROZEN_A',len(files),len(inputs))
