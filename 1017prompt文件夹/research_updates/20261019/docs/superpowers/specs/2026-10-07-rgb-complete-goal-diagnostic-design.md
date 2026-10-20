# 原完整目标：同版本RGB+knownpose受控探索诊断

局部current臂2m实际到达，history候选退化拒绝。原完整目标、可信记忆、估计位姿和世界模型没有由局部结果通过。下一项只运行ONE原golden E1完整起点→原目标35秒诊断，包含原6秒观察；不是重新训练或导航准入后的成功率矩阵。

复用已封存current臂的全部控制与感知实现：FastB0/rig/mask/light/current-source cache/±.3未知主轴cap/rollPID/局部planner与参考规则/physicalcontact_v2，不尝试候选、不换TTLs。唯一任务范围变化为撤销2m截断，恢复原完整goal与episode距离元数据。source仍严格gateREJECT；此运行明确为已批准新任务的风险研究、knownpose特权，不能冒充RGBestimatedpose或正式基线通过。

最多ONE完整35秒、71网络预测；零或失败不得扩大矩阵救分。预封存新入口及原代码/权重/原manifest，全源可追踪；GTdepth/完整map/path仅offline，动作来源不改变。仿真暂停推理，不称实时。

分别评估完整goal终距、物理contact、真实沿目标方向progress/轨迹与各阶段停走、局部reference更新/方向链/动作、来源年龄/unknown与源hazards、runtime latency、视频。沿直线progress不冒充绕障referenceprogress。即使到达也只是该ONE knownpose/风险条件实例，不能称freshformal成功率或世界模型完成。失败保留，不通过延长35秒救原结果。阶段接着做有界阻断来源/感知因果核验，而非返回Oracle调参。

## 在封存前补充的同源接口核验

已完成的同源检查不支持13.5秒纯记忆归因（history最新包current-only也FAIL，current完整前缀history仍PASS）。为去除重置RGB混杂，该ONE完整目标运行增加独立history只读shadow：同一当前/过去真实来源、当前状态/任务goal，独立policy状态各调用一次；shadow请求另存，永不执行、不改变current对象、source/packet/observation由副本传递。用CPU小测试证明输入不被修改、未来包先拒绝、primary on/off请求与边界状态精确同。shadow只是消费接口诊断，没有预测另一个真实轨迹，没有将primary未来轨迹作为shadow不同计划的标签。除taskgoal恢复外，primary行为不变。
