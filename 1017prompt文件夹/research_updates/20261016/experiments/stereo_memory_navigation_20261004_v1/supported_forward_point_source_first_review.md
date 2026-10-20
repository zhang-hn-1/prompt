# 首次限定范围审查

结论：保留 GPU 前拒绝；未发现 Critical 或 Important 问题，发现 2 项 Minor 证据问题。没有进行 GPU、重评稳态门限、重跑历史规划查询或改动实现。

- Minor M1：`preflight_supported_forward_points_v1.py:11` 的封存只绑定脚本自身和五个声明输入，遗漏实际读取的初始姿态 frame_000 与候选来源、规划器、消费代码及相关依赖。当前声明哈希全部匹配，但该封存不能独立证明完整历史 actor 身份。应保留原拒绝并注明遗漏；未来候选需要完整输入/依赖清单。
- Minor M2：覆盖协议同时出现 single72s 任务和 61–65.5 秒窗口，却保留 duration=32、requests=320、captures=321、physics=1601 和旧单方向 sequence。另行封存的 72 秒计划与验证记录一致；应补充明确说明，保留原封存文件及哈希。

个人检查：CPU 定向测试 5 passed in 1.46s；实际枚举只支持 main +.5、raw aux 0/±π/6 三点；保存首边几何复现误差最大 3.36e-15；预检脚本、五个声明输入、旧 BankMap、四个 evaluation source 及 assessment/protocol/verification 链的哈希全部匹配。规划器和消费器差异保留原代价、节点预算、碰撞/retention 与参考行为。

作者证据（与个人检查分开）：原唯一 frame140 查询 unreachable、2 nodes、0 waypoints；三条 .3m 边保存的原 EDT 最小值均 .806225775，小于 .870710678。原 full CPU 日志现为 645 passed in 42.30s，未由审查者重跑。三固定 tail 通过只支持当前三个必要稳态点；正 bank 到达 2.5 秒、反转 4.8 秒，瞬态/全 BankMap/动态模型/新导航未获认证。

未观察到未来标签或 GT 进入 actor、拟合/插值/外推、拒绝后的参数救援或新导航成功声明。上述 Minor 项不改变此次拒绝。
