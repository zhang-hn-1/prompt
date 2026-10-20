"""Fresh identity/append/gates check; no policy/search/simulator reruns."""
from pathlib import Path
import datetime
import hashlib
import json

H = Path(__file__).resolve().parent
W = H.parent.parent
sha = lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
read = lambda name: json.loads((H / name).read_text())


def main():
    p = read('root_direction_protocol.json')
    f = read('root_direction_final_identity.json')
    s = read('root_direction_document_sync.json')
    r = read('root_direction_review.json')
    a = read('root_direction_assessment.json')
    v = read('root_direction_verification.json')
    st = read('root_direction_status.json')
    checks = {}
    for name, mapping in [('sealed_sources', p['source_sha256']), ('old_inputs', p['input_sha256']),
                          ('post_close_actual_outputs', read('root_direction_run/artifact_manifest_post_close.json')['artifact_sha256']),
                          ('final_stage_artifacts', f['final_artifact_sha256']), ('synchronized_documents', s['document_sha256'])]:
        for path, digest in mapping.items():
            assert sha(path) == digest, (name, path)
        checks[name] = len(mapping)
    assert sha(H / 'root_direction_final_identity.json') == s['final_identity_sha256']
    assert sha(H / 'root_direction_verification.json') == s['author_verification_sha256'] == f['author_verification_sha256']
    assert sha(H / 'root_direction_review.json') == s['review_sha256'] == f['review_sha256']
    assert sha(H / 'close_root_direction_stage.py') == s['closure_script_sha256']
    report = W / 'reports/benchmark_root_direction_online_navigation_20261005.md'
    assert sha(report) == f['final_report_sha256']
    assert hashlib.sha256(report.read_bytes().split('\n## 独立审查与作者收尾'.encode())[0]).hexdigest() == r['reviewed_report_sha256'] == f['reviewed_report_body_sha256']
    assert r['review_count'] == 1 and r['critical_count'] == r['important_count'] == 0
    for doc in [W / 'research_mission.md', W / 'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md', H / 'progress.md']:
        assert doc.read_bytes().startswith((H / ('root_direction_pre_close_' + doc.name)).read_bytes())
        assert doc.read_text().count('## 07:44UTC current-direction root cost online — final stage closure') == 1
    assert p['sealed_utc'] < st['start_utc'] < st['end_utc'] < a['assessed_utc'] < v['verified_utc'] < r['saved_utc'] < f['closed_utc'] < s['synchronized_utc']
    assert p['acceptance'] == read('guarded_reference_protocol.json')['acceptance']
    assert st['status'] == 'finished' and st['exit_code'] == 0
    assert p['maximum_sequences'] == p['maximum_new_control_candidates'] == f['new_simulations'] == 1 and not p['baseline_rerun']
    assert v['interface_gate_pass'] and not v['navigation_gate_pass'] and not a['comparison']['limited_improvement_pass']
    assert read('root_direction_handoff_audit.json')['nominal175_exact_previous_candidate']
    assert not read('root_direction_handoff_audit.json')['reverse_primitive_execution_demonstrated']
    assert all(not d['whole_research_complete'] for d in [p, v, a, r, f, s])
    assert f['automation'] == s['automation'] == 'ACTIVE'
    result = dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), checks=checks,
                  reviewed_report_prefix_exact=True, author_document_append_only=True, review_count=1,
                  scope_rulings=len(r['declined_scope_rulings']), actual_run_finished_exit_zero=True,
                  new_simulations=1, baseline_rerun=False, interface_gate_pass=True,
                  limited_improvement_pass=False, navigation_gate_pass=False, reverse_primitive_executed=False,
                  stage_complete=True, whole_research_complete=False, automation='ACTIVE',
                  final_identity_sha256=sha(H / 'root_direction_final_identity.json'),
                  document_sync_sha256=sha(H / 'root_direction_document_sync.json'),
                  scope='Fresh hashes/documents/chronology/gates only; no further scientific runs.')
    (H / 'root_direction_final_check.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
