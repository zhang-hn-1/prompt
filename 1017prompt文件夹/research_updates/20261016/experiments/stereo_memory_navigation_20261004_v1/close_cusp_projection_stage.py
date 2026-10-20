"""Author append-only closure of the bounded CPU stage; no policy/search/sim."""
from pathlib import Path
import datetime
import hashlib
import json

H = Path(__file__).resolve().parent
W = H.parent.parent
REPORT = W / 'reports/benchmark_cusp_projection_regression_20261005.md'
HEADING = '## 08:44UTC unaccepted cusp projection regression — final stage closure'
sha = lambda f: hashlib.sha256(Path(f).read_bytes()).hexdigest()
read = lambda n: json.loads((H / n).read_text())


def save(name, value):
    (H / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def main():
    p = read('cusp_projection_protocol.json')
    a = read('cusp_projection_assessment.json')
    v = read('cusp_projection_verification.json')
    r = read('cusp_projection_review.json')
    pkg = read('cusp_projection_review_package.json')
    assert r['review_count'] == 1 and r['critical_count'] == r['important_count'] == 0
    assert a['projection_cusp_regression_pass'] and a['independent_arm_calls'] == 4
    assert a['new_simulations'] == a['new_planner_searches'] == 0
    assert not a['actual_navigation_candidate_tested'] and not a['real_reverse_completion_demonstrated']
    assert sha(H / 'cusp_projection_assessment.json') == v['assessment_sha256']
    assert sha(H / 'cusp_projection_protocol.json') == v['protocol_sha256']
    body = REPORT.read_bytes().split('\n## 独立审查与作者收尾'.encode())[0]
    assert hashlib.sha256(body).hexdigest() == r['reviewed_report_sha256'] == pkg['reviewed_report_sha256']
    for mapping in [p['source_sha256'], p['input_sha256']]:
        for path, digest in mapping.items():
            assert sha(path) == digest, path
    old = read('root_direction_document_sync.json')
    for path, digest in old['document_sha256'].items():
        archived = H / 'cusp_projection_pre_stage_ledger.md' if Path(path) == H / 'progress.md' else Path(path)
        assert sha(archived) == digest, path
    docs = [W / 'research_mission.md', W / 'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md', H / 'progress.md']
    for path in docs:
        assert HEADING not in path.read_text()
        (H / ('cusp_projection_pre_close_' + path.name)).write_bytes(path.read_bytes())
    excluded = {'cusp_projection_final_identity.json', 'cusp_projection_document_sync.json', 'cusp_projection_final_check.json'}
    files = {f for f in H.glob('*cusp_projection*') if f.is_file() and f.name not in excluded}
    files.add(REPORT)
    ident = dict(closed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), stage_complete=True,
                 whole_research_complete=False, automation='ACTIVE', new_simulations=0, new_planner_searches=0,
                 archived_scientific_policy_calls=4, unique_projection_regression_pass=True,
                 actual_navigation_candidate_tested=False, real_reverse_completion_demonstrated=False,
                 navigation_gate_unchanged_rejected=True, source_checks=len(p['source_sha256']),
                 input_checks=len(p['input_sha256']), reviewed_report_body_sha256=r['reviewed_report_sha256'],
                 final_report_sha256=sha(REPORT), verification_sha256=sha(H / 'cusp_projection_verification.json'),
                 review_sha256=sha(H / 'cusp_projection_review.json'),
                 final_artifact_sha256={str(f): sha(f) for f in sorted(files)})
    save('cusp_projection_final_identity.json', ident)
    text = f'''\n\n{HEADING}

先完整读 `reports/benchmark_cusp_projection_regression_20261005.md` 和H/cusp_projection_protocol.json/assessment/verification/review。上一轮全部来源/产物/文档身份保持，无遗留实际仿真，本阶段只有CPU最小修复+一次归档两时刻对照，无新sim/grid/Hybrid/训练/视频/成功率。Projection regression PASS；实际闭环/反向段完成/导航尚未验证，原35s timeout3.671378/RMS.572390/peak1.105993/0contact以及rootcandidate拒绝保留，较强guarded B0未替换。全研究NOTcomplete/automationACTIVE，常规安静。

唯一修复：已知当前reference方向!=accepted时保持当前idx并返回真实距离，否则调用原projection。原act/handler/five-low .05m/s五10Hz与反弹、速度/PID/BankMap/协议/smoother/planner/rootcost/guard/相机照明保持。正常初始化无pending仍能进入反向交接，资格后恢复原投影；合理新reference可取消过时pending，不永久锁旧路线/不强制提前反转。独立with_cusp_boundary基类仅覆写project，make_cusp_oracle保持actualwrapper组成，CuspProjectionPolicy继承RootDirectionPolicy仅episode reset装配，不action时reset。非法accepted投影拒绝；旧rootplanner入口None Minor仍未修，不把这一层检查冒称全部入口修复。

11真实fixture初callablebaseline7failed4passed RED1.25→11GREEN1.23/fullH195PASS8.72s；双向短F/R/F复现、实际act五低速/反弹/首次无pending/资格release、正常同向完整act/raw/protocol/diag精确、合法geometry replacement/实际MRO/非法None均覆盖，无planner或sim。109源8487旧/case输入于08:53:56.762833 presealed，一次2arms×2currentcalls08:53:58.904760→08:53:58.951319exit0。初态frame180过去9policy资格字段+121pastmapping、175已知参考、181/182当前观测；不是fullphysics/PID/smootherclone。原臂两请求/diag EXACT，181共同状态/动作同，182实际速度.231750候选保持idx1/pendingR/low0/zero，原idx3/pendingNone/main+.5/auxraw-.582204 protocol-.523599。分叉182即结束，不使用183+旧实际轨迹当候选labels；只是raw/protocol改变，不claimapplied/停稳/实际reverse完成。

作者{v['verified_utc']} hash/scalar/chronology/source检查109/8487、4calls/两原请求/共同前缀/新索引层语义与assessmentSHA；没有再act/仿真。决策图是已知参考和共同actualXY，视觉已检查，无候选后续运动。ONEfreshreadonlyreview范围/发现/每条declined裁决见cusp_projection_review.json；审查bodySHA {r['reviewed_report_sha256']}、作者最终报告SHA {sha(REPORT)}；作者append/finalchecker单独归属未二次review。全部源码/case/日志/图/协议保留，无无关Gitcommit/delete；旧sourcegate/M2/RGB/stop拒绝与rig不变。

接续尚未实施的一项实际工作：预封存唯一CuspProjectionPolicy同初态/同6s前缀/原35s goldenE1，对归档root_direction_run仅projection改动因果比较，并同时报告较强guarded B0；胜过更差比较器不是baseline准入。原rootcost/geometry/mapper/PID/speed/guard/five-low/rig保持，首分叉前完整物理/关节/执行/history/SHA核验，之后测真实applied/XY及主轴方向/是否实际通过交接段、独立参考进度/RMS/检查点/goal/contact。CPU修复不保证实际primitive完成；资格后仍用原nearest投影，未证所有短段闭环完成。导航必须goal<=.2原35s0contact，旧限定改善阈值不放宽；失败如实拒绝，不扫参数/网络/重做矩阵。RGB+estimatedpose/唯一学习记忆世界模型/正式E0E1E2尚未准入，本CPUheartbeat不再actualrun，全研究继续ACTIVE。
'''
    for path in docs:
        with path.open('a') as out:
            out.write(text)
    sync = dict(synchronized_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), stage_complete=True,
                whole_research_complete=False, automation='ACTIVE',
                document_sha256={str(f): sha(f) for f in docs + [REPORT]},
                final_identity_sha256=sha(H / 'cusp_projection_final_identity.json'),
                closure_script_sha256=sha(__file__))
    save('cusp_projection_document_sync.json', sync)
    print(json.dumps({k: v for k, v in ident.items() if k != 'final_artifact_sha256'}, indent=2))


if __name__ == '__main__':
    main()
