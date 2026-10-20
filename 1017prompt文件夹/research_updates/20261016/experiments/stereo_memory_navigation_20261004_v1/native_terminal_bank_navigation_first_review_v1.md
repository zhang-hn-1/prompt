# 唯一首次独立审查：terminal-bank consumer v1

审查对象是新 driver/stage 相对冻结 native-flat-pose 父版本的增量。按 requesting-code-review 的 reviewer template 亲自审查，未派生其他审查代理，未改实现或历史 source/test。当前代码不在已提交 Git range 内（目录为 untracked）；本次以冻结父文件及协议 hash 为基线。

## 结论

Critical 0；Important 0；Minor 1。控制增量可进入后续既定流程；实际启动仍须完成 streaming V2 proof 和 author review，这两项不存在属于待完成，不是运行失败。本报告不证明实际运行成功，也不替代 author review。

## 已确认的优点与边界

- Driver 的唯一控制行为差异为第 219 行外包装 `with_terminal_goal_bank`；源、planner、pose、PID、gain、速度、camera/light、fixed14 warmup、完整目标与 60s runner 保持父实现。完整 driver/stage diff 已阅读。
- 新协议 1917 个 source hash 全部匹配；父 1913 个 hash 均原样继承，新增只有新 driver/stage、历史 terminal source、治理 plan。base_protocol_sha256 匹配。full60 actual_acceptance、pose_acceptance 与初始条件全部同父。
- `TerminalGoalBankSource.apply` 在有 main 的当前块只更新过去 motion auxiliary 并原样发出；全 main0 且 fresh request_source 为 goal_braking 时消费先前块。direction_change、goal_reached、local_no_route 等零块保持原样，关闭 active 但保留过去 motion latch。输入拷贝和 main 不变。
- 外包装每次仅调用一次父 act，之后读 tracker.request_diagnostics。真实父 `GuardedReferencePolicy.act` 正常分支调用 tracker.act，无路分支调用 tracker._zero_action 设置 fresh source；无重入、无 tracker stop counter 改写。输出记录同时保留 parent_final_request、final request、latch before/after 和 final source。
- Driver 第 230–241 行使 fixed14 prefix 不进入该 act 外包装；新 start/goal 都是父 `[-2.2,.15,.45]` / `[2.2,.15,.45]`，相同姿态、runtime/camera/light、60s runner。实际物理 prefix 相同须在未来实际运行后验证，代码相同不能替代该证据。
- source_model_admitted=False、模型未准入、initial gauge/lowlevel PID privileged、typed height prior、not formal35 等限制保留。
- stage 第 7–8 行要求 V2 decision 与零 C/I author review、driver/stage hash；第 9、14 行避免重复启动并只运行 R1。未生成前置证据时不能进入 RUNNING；当前没有 stage status、actual output 目录。

## 问题

### Minor M1 — 协议继承字段与本分支身份相互冲突

文件 `native_terminal_bank_navigation_protocol_v1.json:1973,2016,2021`：schema 仍是 `native_flat_pose_navigation_v1`，`only_pose_provider_component_changed=true` 与本分支的 `only_terminal_goal_bank_source_changed=true` 共存。本次 pose/source 明确冻结；历史字段若没有历史作用域容易让下游将它解读为当前干预。改新 schema，并删除该历史 pose-only 断言或显式标记它属于 donor transition；按封存流程更新。此项不改变当前行为或 gate。

Critical：无。Important：无。

## 实际验证

亲自执行：`PYTHONDONTWRITEBYTECODE=1 /home/an/miniconda3/envs/ball_wm/bin/python -m pytest -q -p no:cacheprovider test_terminal_goal_bank_source.py`，在 H 目录；结果 **3 passed in 0.23s**。三个测试覆盖终端限定、过去 latch /恢复/reset/非法值，以及父 act 单调用与 stop counter 不改。

阅读作者 `terminal_bank_source_full_CPU_v1.log` 的结尾：660 passed，3 warnings，46.50s；这只是检查作者证据，**未亲自重跑 660 项**。

阅读 full-event V1 audit 和 assessment：第八个 applied tick 首差 index2783，前七个 applied 相同、执行内部 state 已不同；parent applied/smoothed 每步核对，首差后停止。V1 第 6–9 行预计算全 factualtrace requests，不能称为 streaming 证明，不能用 divergence 后 factual feedback 推断 counterfactual physical trajectory。V2 streaming、future poison、truncate 证明尚未生成，未评定为通过。

## 未检查与 set aside

- 未跑 simulator、未复跑 full-event audit、未检查 candidate 实际物理 prefix /完整轨迹/goal/contact/source/pose/progress/time gate；实际尚未开始。
- Counterfactual physical trajectory 与导航成功：超出共同 feedback 前缀的 execution-clone 范围。
- 实际 full60 验收与 prefix equality：必须由未来那一次实际序列证据决定。
- source-model 科学准入、full3D certified、formal35：治理明确排除，本次保留其否定声明。

后续仍按原治理完成 V2 proof、author review、一次 same-initial conditional full60 实际序列及其原 gate 验证；本次不请求用户审批，不开展重审循环。
