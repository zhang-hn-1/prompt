# 渲染边界第二版：实际几何可见性核验

2026-10-03，新增四次完整运行前冻结。第一版的 typeless 父 prim 上设置 invisible 无效，真实图像仍包含目标球。原四条和第一版四条全部保留，均不称作 clean 无提示运行。

实际 bundled USD CPU 复现：父 prim Imageable=False；父 visibility=invisible，但子球 ComputeVisibility=inherited。本版逐个真实 Imageable 子 prim 设置 invisible，在创建后及每次采样前核验所有标记 ComputeVisibility=invisible，并保存查询值。包含 Start、Goal、OraclePath、ActualPath。不改变机器人、碰撞、相机、灯光、策略或参数。

只重跑相同 B0/C golden E2/E1 四条；同一起点、分叉前动作与状态、视觉回放和未来截断/替换按旧口径核验。属于输入边界修复，非第二个算法候选。历史感知 gate 未通过仍保留，不称为本版本 fresh 感知验收。
