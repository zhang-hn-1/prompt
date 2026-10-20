# 强双眼历史核心记忆：来源覆盖通过，规划消费者拒绝

唯一候选 REJECT_UNIQUE_PERSISTENT_BILATERAL_CORE_MEMORY。全部29个0–14秒源更新输入，4个pending后25个推导窗口累计837个原raw核心，统一.20 inflation一次，220个XY可能格/2.20m²；真实footprint185/185，200个贡献class视图最小precision0.9929367，来源预检查通过。数据用于设计，属于历史，不能称fresh/online。

冻结planner在14秒当前估计起点的栅格SDF0.8062258<0.8707107，route和local kinodynamic均拒绝；没有发起新的导航/GPU，也没有降低margin或选择有利窗口。当前末窗口自身SDF1.2041595、168格；阻塞确实由累计新增历史MODEL导致，不把来源覆盖通过称作导航通过。

独立离线因果核验：只有一个阻塞栅格，由raw核心(-0.9669953,0.0927105,0.75)经.2扩展和栅格化贡献。它只在frame100/10秒的四曝光推导窗口中出现，与真实footprint相距0.6170383m。左眼frame85有未遮挡分类正支持，但核心opticalZ0.9891215，实测网络深度1.6430204/confidence=true，渲染诊断深度1.6524779。右眼直到frame100才有未遮挡分类正支持，其核心Z0.9658299，渲染诊断1.7162189。说明跨时间双眼颜色支持没有证明同一三维点深度。高classprecision不能替代metric geometry。未用这些GT删除actor点或重算候选成绩。

实际个人运行：4个核心TDD RED缺模块→GREEN，1个独立seed合同RED→GREEN；618全套39.37秒。首次限定review本人5tests1.26秒，C0/I2/M0：NaN当前时钟和fractional frame截断会绕过身份合同。作者一次修复，2 RED/5PASS→全套620PASS39.16秒，旧源码字节保存 *_before_first_review_fix_v1.py.txt；实际合法数据不存在这两异常，历史拒绝不重新计分。预检查v1会话58458退出1（写死14秒与真实13.999999687时钟不符），新v2改用实际pose时刻/yaw，会话14669退出0，v1输出/log保留。阻塞审计96167退出0，单核心投影审计退出0。本人查看实际阻塞图与前一摇摆图。

MODEL计算/池出生约14秒与分类模型出生约2秒分别保留。历史支持窗口时刻是推导窗口，不是逐点测量时刻；静态持续先验显式标注，measured/free均false，unknown和安全起步/平地先验另标。源码、封存pool、逐核心来源、200视图、planner前真值结果、独立标签评分在 bilateral_core_memory_preflight_v2；审计在 persistent_core_blocker_audit_v1 和 blocking_core_projection_audit_v1.json。原pool拒绝、摇摆末端拒绝、D4来源拒绝及oldNN/学习拒绝保留。原35秒E0/E1/E2仍未触发，整体研究未完成。

下一项针对来源几何歧义，而不是换记忆窗口或控制：只检查一个实测双目metric支持与历史颜色核心的一致性结构，保持旧NN冻结与未准入。它是独立来源候选，不能回改本pool拒绝；先冻结门槛与标签隔离再执行，失败不扫threshold/网络。仅根据封存证据决定是否准入新的在线闭环。
