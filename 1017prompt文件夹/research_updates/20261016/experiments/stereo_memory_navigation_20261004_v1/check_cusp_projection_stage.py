"""Fresh author closure hashes/append/chronology/gates only."""
from pathlib import Path
import datetime
import hashlib
import json

H = Path(__file__).resolve().parent
W = H.parent.parent
sha = lambda f: hashlib.sha256(Path(f).read_bytes()).hexdigest()
read = lambda n: json.loads((H / n).read_text())


def main():
    p = read('cusp_projection_protocol.json'); a = read('cusp_projection_assessment.json')
    v = read('cusp_projection_verification.json'); r = read('cusp_projection_review.json')
    f = read('cusp_projection_final_identity.json'); s = read('cusp_projection_document_sync.json')
    checks = {}
    for name, mapping in [('sealed_sources', p['source_sha256']), ('old_and_case_inputs', p['input_sha256']),
                          ('final_stage_artifacts', f['final_artifact_sha256']), ('synchronized_documents', s['document_sha256'])]:
        for path, digest in mapping.items():
            assert sha(path) == digest, (name, path)
        checks[name] = len(mapping)
    assert sha(H / 'cusp_projection_final_identity.json') == s['final_identity_sha256']
    assert sha(H / 'close_cusp_projection_stage.py') == s['closure_script_sha256']
    assert sha(H / 'cusp_projection_assessment.json') == v['assessment_sha256']
    assert sha(H / 'cusp_projection_protocol.json') == v['protocol_sha256']
    assert sha(H / 'cusp_projection_review.json') == f['review_sha256']
    assert sha(H / 'cusp_projection_verification.json') == f['verification_sha256']
    report = W / 'reports/benchmark_cusp_projection_regression_20261005.md'
    assert sha(report) == f['final_report_sha256']
    assert hashlib.sha256(report.read_bytes().split('\n## 独立审查与作者收尾'.encode())[0]).hexdigest() == r['reviewed_report_sha256'] == f['reviewed_report_body_sha256']
    for path in [W / 'research_mission.md', W / 'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md', H / 'progress.md']:
        assert path.read_bytes().startswith((H / ('cusp_projection_pre_close_' + path.name)).read_bytes())
        assert path.read_text().count('## 08:44UTC unaccepted cusp projection regression — final stage closure') == 1
    assert r['review_count'] == 1 and r['critical_count'] == r['important_count'] == 0
    assert p['sealed_utc'] < a['started_utc'] < a['assessed_utc'] < v['verified_utc'] < r['saved_utc'] < f['closed_utc'] < s['synchronized_utc']
    assert p['maximum_scientific_policy_calls'] == a['independent_arm_calls'] == 4
    assert a['archived_tracker_requests_exact'] == 2 and a['first_current_request_divergence_frame'] == 182
    assert a['new_simulations'] == a['new_planner_searches'] == f['new_simulations'] == f['new_planner_searches'] == 0
    assert a['projection_cusp_regression_pass'] and f['unique_projection_regression_pass']
    assert not a['real_reverse_completion_demonstrated'] and not a['actual_navigation_candidate_tested']
    assert all(not d['whole_research_complete'] for d in [p, a, v, r, f, s])
    assert f['automation'] == s['automation'] == 'ACTIVE'
    result = dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), checks=checks,
                  reviewed_prefix_exact=True, author_documents_append_only=True, review_count=1,
                  scope_rulings=len(r['declined_scope_rulings']), archived_policy_calls=4,
                  new_simulations=0, new_searches=0, projection_cpu_regression_pass=True,
                  real_reverse_completion_demonstrated=False, navigation_gate_unchanged_rejected=True,
                  stage_complete=True, whole_research_complete=False, automation='ACTIVE',
                  final_identity_sha256=sha(H / 'cusp_projection_final_identity.json'),
                  document_sync_sha256=sha(H / 'cusp_projection_document_sync.json'))
    (H / 'cusp_projection_final_check.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
