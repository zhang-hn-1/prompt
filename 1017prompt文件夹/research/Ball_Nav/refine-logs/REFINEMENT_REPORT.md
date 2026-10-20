# Refinement Report

**Problem**: 移动机器人运动-视角耦合导致ImageGoal导航失效
**Date**: 2026-05-18
**Rounds**: 3 / 5
**Final Score**: 9.1 / 10
**Final Verdict**: READY

## Score Evolution

| Round | 问题保真 | 方法具体性 | 贡献质量 | Frontier | 可行性 | 验证聚焦 | TRO就绪 | Overall | Verdict |
|-------|---------|-----------|---------|---------|--------|---------|---------|---------|---------|
| 1     | 8.5     | 5.0       | 5.5     | 7.0     | 5.5    | 6.5     | 6.0     | 6.1     | REVISE  |
| 2     | 9.5     | 8.1       | 8.8     | 8.0     | 7.8    | 8.0     | 8.0     | 8.4     | NOT READY |
| 3     | 9.2     | 9.1       | 8.8     | 9.0     | 9.0    | 9.4     | 9.0     | 9.1     | **READY** |

## Final Proposal Snapshot
- 见 `refine-logs/FINAL_PROPOSAL.md`
- 核心论文论题（5条）：
  1. 显式视角潜状态 r_t ∈ R⁴（sin-cos，相对起始帧）解决rollout视角传播问题
  2. Ray-conditioned token interaction将视角变换内嵌到token几何中
  3. K=3 closed-loop overshooting训练对齐MPPI推理行为
  4. DUSt3R离线教师（置信度加权）提供高质量伪标签，零推理开销
  5. 转弯强度分层实验设计（高/低转弯路线）提供精准的因果验证

## Method Evolution Highlights
1. **最重要的聚焦动作**（Round 1→2）：将"Δθ辅助估计器"升级为"显式潜状态r_t + 联合转移模型"——从辅助技巧变为核心机制
2. **最重要的机制升级**（Round 1→2）：Positional bias → Ray-conditioned token interaction，表达力从"偏移注意力权重"到"物理射线几何"
3. **最重要的训练对齐**（Round 2→3）：One-step损失 → K-step closed-loop overshooting，消除train-test mismatch这一根本性blocking

## Pushback / Drift Log
| Round | Reviewer Said | Author Response | Outcome |
|-------|--------------|----------------|---------|
| 2 | 加入full SO(3)建模 | 拒绝：只建模平台主要DoF，避免过度参数化 | Accepted |
| 2 | 扩大Claim 2为零样本导航 | 拒绝：保持为表示迁移（转弯预测误差），避免controller mismatch混淆 | Accepted |
| 3 | 进一步扩大transfer范围 | 拒绝：drift风险，保持当前保守声明 | Accepted |

## Remaining Weaknesses（诚实评估）
1. 真实机器人实验数量（500条轨迹）是否足够，取决于场景多样性
2. DUSt3R在快速旋转/运动模糊段的稳定性仍需实验验证
3. K-step overshooting在球型机器人独特动力学下的收敛性待验证
4. HuRoN四足数据集的相机外参差异可能影响transfer实验

## Next Steps
- **READY → 进入 /experiment-plan**：将本方案转化为详细实验路线图（数据采集协议、模型实现、评测脚本）
- 论文题目建议：**"EV-WM: Explicit Viewpoint State for Viewpoint-Coupled Visual Navigation via Latent World Models"**
