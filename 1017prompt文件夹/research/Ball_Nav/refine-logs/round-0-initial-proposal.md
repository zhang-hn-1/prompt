# Research Proposal: Motion-Coupled Visual World Model for ImageGoal Navigation

## Problem Anchor
- **Bottom-line problem**: 移动机器人（球型机器人/机械狗/无人机）在导航过程中，运动与视觉观测强耦合——体态变化（滚动、步态、姿态）持续改变摄像头视角，导致现有视觉世界模型无法准确预测"执行某动作后会看到什么"，ImageGoal导航规划失效。
- **Must-solve bottleneck**: 现有潜空间世界模型（DINO-WM系列）在预测未来视觉特征时，将动作映射到"场景位移"，完全忽略了机器人体态变化对视角的贡献。转向动作既改变位置，又改变视角朝向，两者耦合导致特征预测系统性偏差。
- **Non-goals**: 不做VLN/语言导航；不做像素级视频生成；不做SLAM建图；不做端到端像素→动作。
- **Constraints**: 双目RGB摄像头；机器人上GPU（显存有限，推理需轻量）；GPU服务器训练；真实机器人数据可采集；6个月投TRO。
- **Success condition**: 真实球型机器人上ImageGoal导航成功率明显优于忽略视角耦合的基线（DINO-WM等）；在机械狗/无人机平台有迁移验证证明通用性。

---

## Technical Gap

### 现有方法的失效点

**DINO-WM**（ICML 2025）：
- 冻结DINOv2编码器提取patch特征，ViT预测器预测未来patch特征
- 动作输入为 [v, ω] 线速度和角速度
- 隐含假设：动作→场景中的位移，视角方向固定或变化微小
- 失效：球型机器人转向时，摄像头视角同步旋转，同一场景的DINOv2特征分布完全改变，预测器学到的映射完全错误

**NWM**（CVPR 2025）：
- 像素级视频生成，1B参数
- 机器人上GPU跑不动，不适合部署

**ANWM**（无人机，arXiv:2512.21887）：
- 用Future Frame Projection（FFP）：深度图+相机位姿→3D点云→几何投影预测未来视角
- 问题1：依赖精确深度和位姿估计，误差累积
- 问题2：像素级预测，计算重
- 问题3：只针对无人机4-DoF，不通用

### 为什么简单扩展不够

- 直接给DINO-WM加位姿信息：位姿噪声大，且几何投影需要深度，仍然计算重
- 直接做数据增强（随机旋转视角）：不能建模运动-视角的物理耦合关系
- 用像素级世界模型：机器人GPU跑不动

### 最小必要机制

在潜空间内，**联合建模运动引起的场景变化和视角变化**，用轻量的学习机制替代显式几何投影。

---

## Method Thesis
- **One-sentence thesis**: 通过在DINO特征空间内引入"视角感知转换层"，将动作分解为场景位移分量和视角旋转分量，使潜空间世界模型能准确预测视角耦合运动下的未来视觉特征，实现鲁棒的ImageGoal导航。
- **Why smallest adequate**: 只需在现有DINO-WM预测器中加入一个可学习的视角感知机制，不需要深度传感器、显式3D重建或像素生成。
- **Why timely**: DINOv2的patch特征具有良好的空间结构，天然适合建模视角变换；双目相机提供自然的几何约束，无需额外传感器。

---

## Contribution Focus
- **Dominant contribution**: 视角感知潜空间世界模型（Viewpoint-Aware Latent World Model, VA-LWM）——在DINO特征空间内联合建模动作的场景位移效应和视角旋转效应。
- **Optional supporting contribution**: 双目视差引导的视角估计——利用双目相机的几何约束提供轻量的视角变化监督信号，替代显式深度+位姿估计。
- **Explicit non-contributions**: 不提出新的规划算法（复用MPPI）；不提出新的特征编码器（复用DINOv2）；不做像素级生成。

---

## Proposed Method

### Complexity Budget
- **Frozen/reused**: DINOv2编码器（完全冻结）；MPPI规划器（标准实现）；底层PPO控制器（已有）
- **New trainable components**:
  1. 视角感知预测器（Viewpoint-Aware Predictor）：在ViT预测器中加入视角旋转感知注意力，参数增量小
  2. 视角变化估计器（Viewpoint Change Estimator）：从双目图像序列估计相对视角变化，轻量MLP
- **Intentionally excluded**: 深度网络；显式3D点云；像素解码器；VLM；大规模预训练

### System Overview

```
双目RGB图像 (左/右)
      |
      v
[冻结DINOv2编码器]
      |
左目patch特征 z_t ∈ R^{N×E}
      |                    \
      |              [视角变化估计器]
      |              双目视差 → Δθ_t（视角变化量）
      |                    /
      v
[视角感知预测器 VA-Predictor]
输入：z_{t-H:t}，a_{t-H:t}，Δθ_{t-H:t}
输出：ẑ_{t+1}（预测未来patch特征）
      |
      v
[MPPI规划器]
候选动作序列 → 展开预测 → 与目标图像特征比较
      |
最优动作序列
      |
      v
[底层PPO控制器（已有）]
电机指令 @ 50Hz
```

### Core Mechanism：视角感知预测器

**核心思想**：将动作的效应分解为两个正交分量：

```
动作 a_t = [v, ω]
  → 场景位移效应：机器人向前移动，场景内容变化
  → 视角旋转效应：机器人转向，摄像头朝向变化

传统世界模型只建模场景位移效应
本文联合建模两者
```

**具体实现**：

在ViT预测器的注意力层中，加入视角旋转感知的位置编码调制：

```python
# 标准DINO-WM预测器
z_pred = ViT_predictor(z_history, action_history)

# VA-LWM：加入视角感知
delta_theta = viewpoint_estimator(left_img, right_img)  # 从双目估计视角变化
z_pred = VA_ViT_predictor(z_history, action_history, delta_theta_history)
```

VA-ViT预测器的关键修改：
- 在每个注意力层加入视角旋转位置偏置（Rotary Position Embedding的2D扩展）
- 视角变化 Δθ 调制patch之间的相对注意力权重
- 参数增量：~5% 额外参数

**训练信号**：
```
L_total = L_pred + λ · L_viewpoint

L_pred：预测特征 ẑ_{t+1} 与真实特征 z_{t+1} 的L2距离（主损失）
L_viewpoint：视角变化估计的一致性损失（辅助）
  = ||Δθ_estimated - Δθ_geometry||²
  其中 Δθ_geometry 由双目视差几何计算（作为弱监督）
```

### Supporting Component：双目视角变化估计器

**为什么需要**：视角变化 Δθ 如果从IMU估计会有漂移，直接从双目图像估计更稳定。

**实现**：轻量CNN提取左右目特征，估计相对视角变化：
```
[左目特征, 右目特征] → 小型CNN → Δθ ∈ R³（俯仰/偏航/滚转）
```
用双目几何约束（三角测量得到的场景点对应关系）提供自监督信号，无需人工标注。

**为什么不是contribution sprawl**：这个组件是为主要贡献（VA-Predictor）提供监督信号的辅助模块，不作为独立贡献声明。

### Training Plan

**数据采集**（真实机器人）：
- 在平地/砾石/室内等场景采集500-1000条轨迹
- 每条轨迹记录：[左目图像序列, 右目图像序列, IMU, 电机指令]
- 采集时复用已有PPO策略，覆盖直行/转弯/加速等多种运动模式

**训练阶段**：
- Stage 1：预训练视角变化估计器（双目自监督，独立训练）
- Stage 2：联合训练VA-预测器（冻结DINOv2，训练VA-ViT + 视角估计器）
- Stage 3：MPPI规划微调（冻结世界模型，调整规划超参）

**训练量**：
- 500条轨迹 × 平均100帧 = 50,000个训练样本（足够）
- ViT-S规模预测器：~22M参数，GPU服务器约4-8小时收敛

### Failure Modes and Diagnostics

| 失效模式 | 检测方式 | 缓解方案 |
|---------|---------|---------|
| 视角估计不准（快速旋转）| 监测 Δθ 估计误差 | 用IMU辅助，加运动平滑 |
| 长时序预测误差累积 | 监测预测特征与观测特征的距离 | 限制规划horizon，定期重新观测 |
| 新环境泛化差 | 测试集成功率下降 | DINOv2冻结特征本身泛化强 |

### Novelty and Elegance Argument

**最近的竞争对手**：
- DINO-WM（ICML 2025）：没有视角感知，在视角耦合平台上失效
- ANWM（arXiv 2025）：有视角感知，但依赖像素级生成+显式几何投影，计算重
- ReL-NWM（2025）：轻量潜空间，但无视角感知

**本工作的差异**：
- 首个在潜空间内（不需像素生成）建模视角-运动耦合的世界模型
- 用双目几何约束提供视角变化的弱监督，无需深度传感器或IMU精确标定
- 方法通用：适用于任何摄像头视角随运动变化的平台

**为什么不是module pile-up**：
- 核心改动只有一处：VA-ViT预测器的视角感知注意力机制
- 视角估计器是为训练服务的辅助模块，推理时可用IMU粗估计替代
- 整体参数增量 < 10%

---

## Claim-Driven Validation Sketch

### Claim 1（主要）：视角感知预测器显著提升视角耦合场景下的预测精度和导航成功率

- **Minimal experiment**：在球型机器人真实数据上对比 VA-LWM vs DINO-WM vs ReL-NWM，评估特征预测误差（MSE）和ImageGoal导航成功率
- **Baselines**: DINO-WM（无视角感知）、ReL-NWM（无视角感知）、ANWM（像素级视角感知，重量级）
- **Metric**: 导航成功率SR（主）、特征预测MSE（辅）、推理延迟（辅）
- **Expected evidence**: 在转弯多的场景（高视角变化），VA-LWM vs DINO-WM SR差距 > 20%；推理延迟优于ANWM 5x以上

### Claim 2（支撑）：方法通用，可迁移到其他视角耦合平台

- **Minimal experiment**：在公开的机械狗导航数据集（HuRoN/ReCon）上，无需重新设计，直接验证
- **Baselines**: 同上，在相同数据上对比
- **Metric**: SR、SPL
- **Expected evidence**: VA-LWM在机械狗数据上同样优于DINO-WM基线

### Ablation（消融）

| 消融项 | 验证的必要性 |
|--------|------------|
| 去掉视角感知注意力 → 退化为DINO-WM | 证明视角感知机制的必要性 |
| 去掉双目监督 → 只用IMU | 证明双目几何约束的价值 |
| 视角感知 vs 几何投影（ANWM方式）| 证明潜空间方式优于像素级方式 |

---

## Experiment Handoff Inputs
- **Must-prove claims**: Claim 1（球型机器人上的SR提升）
- **Must-run ablations**: 去掉视角感知注意力；双目监督的必要性
- **Critical datasets/metrics**: 球型机器人真实数据（自采集）；HuRoN/ReCon公开数据集；SR/SPL
- **Highest-risk assumptions**: 视角变化估计的精度是否足够驱动预测器学习

---

## Compute & Timeline Estimate
- **GPU-hours**: 数据采集1周；Stage1训练4h；Stage2训练8h；实验评测40h；合计约60 GPU-hours
- **Data cost**: 球型机器人真实轨迹采集（人工操控）约2-3天
- **Timeline**:
  - 月1：数据采集 + 框架搭建
  - 月2：模型训练 + 仿真验证
  - 月3：真实机器人实验
  - 月4：对比实验 + 消融
  - 月5：论文写作
  - 月6：修改投稿
