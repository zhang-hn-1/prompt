# Review Summary

**Problem**: 移动机器人运动-视角耦合导致潜空间世界模型ImageGoal导航失效
**Initial Approach**: 视角感知潜空间世界模型（Δθ辅助的DINO-WM扩展）
**Date**: 2026-05-18
**Rounds**: 3 / 5
**Final Score**: 9.1 / 10
**Final Verdict**: READY

## Problem Anchor
移动机器人（球型/四足/无人机）体态变化持续改变摄像头视角，导致DINO-WM类模型MPPI rollout时无法传播视角演化，在转弯主导场景系统性失败。目标：轻量潜空间世界模型 + 显式视角状态 + multi-step viewpoint-consistent rollout。

## Round-by-Round Resolution Log

| Round | 主要审稿问题 | 本轮修改 | 解决状态 |
|-------|------------|---------|---------|
| 1 (6.1) | CRITICAL: 单帧双目≠时序Δθ；rollout无法预测未来视角；positional bias表达力不足 | 引入显式视角潜状态r_t + ray-conditioned token interaction + DUSt3R离线教师 | 部分解决 |
| 2 (8.4) | BLOCKING: 训练one-step vs推理multi-step不匹配；r_t坐标系未定义 | 加入K=3 closed-loop overshooting训练；精确定义r_t（sin-cos，相对起始帧）；删除ray_alignment_loss；伪标签置信度加权 | 完全解决 |
| 3 (9.1) | Non-blocking: 视角漂移诊断缺失；坐标约定细节；过滤数据统计 | 确认READY；加入诊断实验计划；收窄novelty声明 | READY |

## Final Status
- **Anchor status**: 完整保留
- **Focus status**: 聚焦（一个主贡献 + 一个支撑贡献）
- **Modernity status**: 适当使用frontier原语（DINOv2 + DUSt3R + cosine loss）
- **Strongest parts**: 显式视角状态定义精确；K-step closed-loop训练对齐推理；实验设计转弯强度分层
- **Remaining risks**: 真实实验数据质量；DUSt3R在极端运动下的稳定性；transfer claim的严格性
