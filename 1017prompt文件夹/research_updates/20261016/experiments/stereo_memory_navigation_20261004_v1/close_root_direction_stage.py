"""Append-only reviewed stage closure; hashes only, no science rerun."""
from pathlib import Path
import datetime
import hashlib
import json

H = Path(__file__).resolve().parent
W = H.parent.parent
REPORT = W / 'reports/benchmark_root_direction_online_navigation_20261005.md'
PLAN = W / 'docs/superpowers/plans/2026-10-05-root-direction-online.md'
SPEC = W / 'docs/superpowers/specs/2026-10-05-root-direction-online-design.md'
HEADING = '## 07:44UTC current-direction root cost online — final stage closure'
APPENDIX = '\n## 独立审查与作者收尾'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(name):
    return json.loads((H / name).read_text())


def save(name, value):
    (H / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def main():
    p = read('root_direction_protocol.json')
    v = read('root_direction_verification.json')
    a = read('root_direction_assessment.json')
    st = read('root_direction_status.json')
    r = read('root_direction_review.json')
    pkg = read('root_direction_review_package.json')
    assert r['review_count'] == 1 and r['critical_count'] == r['important_count'] == 0
    assert v['interface_gate_pass'] and not v['navigation_gate_pass']
    assert not a['comparison']['limited_improvement_pass']
    assert st['status'] == 'finished' and st['exit_code'] == 0
    assert p['maximum_sequences'] == p['maximum_new_control_candidates'] == 1
    assert p['acceptance'] == read('guarded_reference_protocol.json')['acceptance']
    body = REPORT.read_bytes().split(APPENDIX.encode())[0]
    assert hashlib.sha256(body).hexdigest() == r['reviewed_report_sha256'] == pkg['reviewed_report_sha256']
    for mapping in [p['source_sha256'], p['input_sha256'], read('root_direction_run/artifact_manifest_post_close.json')['artifact_sha256']]:
        for path, digest in mapping.items():
            assert sha(path) == digest, path
    previous = read('direction_root_cost_document_sync.json')
    for path, digest in previous['document_sha256'].items():
        restored = H / 'root_direction_pre_stage_ledger.md' if Path(path) == H / 'progress.md' else Path(path)
        assert sha(restored) == digest, path
    docs = [W / 'research_mission.md', W / 'docs/superpowers/plans/2026-10-04-stereo-memory-navigation.md', H / 'progress.md']
    for doc in docs:
        assert HEADING not in doc.read_text()
        (H / ('root_direction_pre_close_' + doc.name)).write_bytes(doc.read_bytes())
    excluded = {'root_direction_final_identity.json', 'root_direction_document_sync.json', 'root_direction_final_check.json'}
    files = {path for path in H.glob('*root_direction*') if path.is_file() and path.name not in excluded}
    files.update([REPORT, PLAN, SPEC])
    identity = dict(closed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), stage_complete=True,
                    whole_research_complete=False, automation='ACTIVE', interface_gate_pass=True,
                    limited_improvement_pass=False, navigation_gate_pass=False, new_simulations=1,
                    baseline_rerun=False, reverse_primitive_executed=False,
                    source_checks=len(p['source_sha256']), input_checks=len(p['input_sha256']),
                    post_close_checks=len(read('root_direction_run/artifact_manifest_post_close.json')['artifact_sha256']),
                    reviewed_report_body_sha256=r['reviewed_report_sha256'], final_report_sha256=sha(REPORT),
                    author_verification_sha256=sha(H / 'root_direction_verification.json'),
                    review_sha256=sha(H / 'root_direction_review.json'),
                    final_artifact_sha256={str(path): sha(path) for path in sorted(files)})
    save('root_direction_final_identity.json', identity)
    text = f'''\n\n{HEADING}

先完整读 `reports/benchmark_root_direction_online_navigation_20261005.md` 和冻结spec/plan `docs/superpowers/{{specs,plans}}/2026-10-05-root-direction-online*.md`。ONE同起点goldenE1实际原35s于07:54:40.229131—07:57:32.904552UTC结束exit0，无遗留仿真/第二候选，归档guarded B0不重跑。接口PASS，事前限定改善REJECT，导航REJECT(timeout35s/goalremain3.671378/0contact)。全研究NOTcomplete/automationACTIVE，常规安静，不替换已归档B0。

只有原guard rebuild向独立planner传PRE-planning accepted direction并应用旧.5rootcost；初次None，后11次forward；retention不搜索/reset。原reverse1.25/node3000/primitive.3/.05sampling/goal.2/localtarget2.4/crop±4/clearance.6、PID/speed/rig/light/协议/smoother/五低速反弹均冻结。真实depth+knownpose显式unknownrisk，不是RGB/safe/estimatedpose，外层privilegedFalse只拒绝全图/全局Oracle在线输入。无footprintfree/raycarving/GTfilter/futuretruth/新模型或参数扫描。

106源/5670旧输入07:54:23.300306 presealed，350请求351双方capture1751physics2101可用接触0，两视频351帧10fps无尾步/markershidden/0age。关闭后2762manifest含bankfeedback，原manifest pre-close范围单列。首分叉175/17.5s，175request876physics17nonraw/176execution+packet history EXACT，raworigin+.02s/+1tick初始化差异独立；116rawblocks738appliedticks改变。实际175 reference EXACT先前名义candidate，58boundaries46retain12rebuildALLsuccess，source类型/安装/继承act/reset和retention状态明确。独立评价reference进度4.855576→5.389775(+.534199)，goalremain4.156980→3.671378(-.485602)，但RMS.174067→.572390/peak.475266→1.105993，故预gate拒绝；4m空间点25.3→19.7s，6/8未实际通过，不把索引或停车当成功。

只读完整日志分段定位继承projection接口：frame181idx0F→idx1R pendingR/low0，182在accepted仍F/未资格时窗口放行下一forward段→idx3F、pending取消；version3未替换。17.5→23 raw/applied/mainjoint均无negative，最低applied+.45、mainjoint+.418757/root纵向速度+.161221，短reverse未实际执行。候选brake.1s不能解释为比B0合法3.8s换向更好。同期自己冻结version3 RMS.260636/peak.510890偏离；后来稳定local23→27/30.5→35 RMS.035565/.022107，但独立全参考末态偏离1.086392，不能把全部失败归首次交接/PID，也不能以local小误差认证导航。

六feature RED1.81→GREEN2.53/184全Htests8.45s。作者{v['verified_utc']} 106源/5670旧输入/2762产物SHA、350causal policy/map/reference/request/diag、351双方depthpacket、350单次history shift、1745实测feedback执行链精确，首5ticks不克隆/非递归物理预测；3finite-past checkpoints futuretrap/truncate一致不是3完整freshphysicsruns。初assessment pending保留，后verification接口PASS/导航REJECT，旧gate不改。ONE独立只读review及每条declined裁决见root_direction_review.json；审查bodySHA {r['reviewed_report_sha256']}，作者最终报告SHA {sha(REPORT)}，作者append/最终hashcheck不冒称二次审查。全部实际代码/数据/日志/视频/版本保留，无无关Gitcommit/delete。

接续尚未实施的一项工作：CPU精确复现未资格换向时投影越过短reverse段，核验waypoint出段/accepted/pending/窗口调用顺序；只设计一个最小投影方向边界修复，保持原五次低速/反弹/mapper/PID/speed/planner/rig/gates，不强制提前反转或循环等待。真实fixture验证/身份与预sealed独立进度/横向/目标/contact gate后，才在后续独立阶段考虑最多一条同初态35s对照，本heartbeat不新增仿真/搜索/调参。所有RGB/sourceconsumer/M2/旧停止包络拒绝保留；估计位姿/唯一学习记忆世界模型/正式35s E0E1E2未准入，全研究继续ACTIVE。
'''
    for doc in docs:
        with doc.open('a') as out:
            out.write(text)
    sync = dict(synchronized_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), stage_complete=True,
                whole_research_complete=False, automation='ACTIVE',
                document_sha256={str(path): sha(path) for path in docs + [REPORT, PLAN]},
                final_identity_sha256=sha(H / 'root_direction_final_identity.json'),
                author_verification_sha256=sha(H / 'root_direction_verification.json'),
                review_sha256=sha(H / 'root_direction_review.json'), closure_script_sha256=sha(__file__))
    save('root_direction_document_sync.json', sync)
    print(json.dumps({k: v for k, v in identity.items() if k != 'final_artifact_sha256'}, indent=2))


if __name__ == '__main__':
    main()
