# Round 1 Refinement

## Problem Anchor（不变，逐字复制）
- **Bottom-line problem**: 移动机器人（球型/四足/无人机）在导航中运动与视觉观测强耦合——体态变化持续改变摄像头视角，导致现有视觉世界模型无法准确预测"执行某动作后会看到什么"，ImageGoal导航规划失效。
- **Must-solve bottleneck**: DINO-WM类模型将动作映射到场景位移，完全忽略视角旋转。转向动作既改位置又改朝向，导致特征预测系统性偏差。
- **Non-goals**: No VLN; no pixel-level generation; no SLAM; no end-to-end pixel→action.
- **Constraints**: 双目RGB；机器人GPU显存有限；GPU服务器训练；可采集真实数据；6个月投TRO。
- **Success**: 真实球型机器人SR明显优于忽略视角耦合的基线；迁移验证证明通用性。

---

## Anchor Check
- **原始瓶颈**：运动-视角耦合导致潜空间预测失效
- **修订方法是否仍解决该问题**：是。修订后方法更精确地建模视角状态转移，更直接地解决原始问题
- **拒绝的reviewer建议（会造成drift）**：
  - 加入像素级重建或3D点云 → drift，明确拒绝
  - 将贡献转移到"弱里程计估计" → drift，拒绝

---

## Simplicity Check
- **修订后主贡献**：显式视角潜状态 + 联合视角-场景转移模型（Joint Viewpoint-Scene Transition Model）
- **删除的组件**：独立的CNN双目视角估计器（合并入轻量位姿头，附着在DINOv2特征上）
- **拒绝的复杂化建议**：不加入深度重建；不加入完整SO(3)旋转建模（只建模主要DoF）
- **为什么仍然是最小充分机制**：核心改动仍然只有一处——在潜空间内加入显式视角状态，用ray-conditioned attention替代bias，参数增量仍然很小

---

## Changes Made

### 1. CRITICAL: 视角从辅助估计升级为显式潜状态

**Reviewer说**：单帧双目视差不能提供时序Δθ；MPPI rollout需要预测未来视角，但方案没有定义。

**Action**：引入**显式视角潜状态** r_t，定义联合转移：
```
(z_t, r_t, a_t) → (ẑ_{t+1}, r̂_{t+1})
```
- r_t ∈ R^d 是紧凑的视角状态向量（不是原始欧拉角）
- 训练时从离线几何教师获取r_t伪标签（见下）
- 推理时r_t由轻量位姿头在线更新（可用IMU辅助）
- MPPI rollout时：模型同时预测下一帧特征 AND 下一帧视角状态，实现连贯的视角感知规划

**Reasoning**：这直接解决了"rollout时未来视角未定义"的根本问题，且不增加像素级计算。

**Impact**：方案核心从"Δθ辅助的预测器"升级为"视角-场景联合转移模型"，贡献更清晰。

---

### 2. CRITICAL: 用Ray-Conditioned Token Interaction替代Positional Bias

**Reviewer说**：Δθ调制的positional bias太弱，无法解释token重索引、遮挡和透视变化。

**Action**：将每个DINO patch token附加**相机射线嵌入**（camera ray embedding）：
```
token_i' = concat(z_i, ray_i(r_t))
```
- ray_i(r_t)：第i个patch对应的相机射线方向，由当前视角状态r_t旋转得到
- 在跨时间步的cross-attention中，用旋转后的ray嵌入计算token匹配
- 效果：模型"知道"每个token对应场景中的哪个方向，视角旋转后token自动对应到新的场景方向

这比pure positional bias更物理合理，且仍然轻量（ray嵌入是低维向量）。

**Reasoning**：这是现代几何感知token conditioning的标准做法，符合frontier-aware要求。

**Impact**：核心机制的表达能力大幅提升，贡献质量分预计从5.5提升到7+。

---

### 3. IMPORTANT: 视角伪标签来源改为离线几何教师

**Reviewer说**：单帧双目视差弱监督在低纹理场景会失效；建议用DUSt3R/MASt3R类方法离线产生伪标签。

**Action**：
- **离线阶段**：用DUSt3R等多视图几何基础模型对采集的真实轨迹数据进行离线处理，产生高质量视角状态伪标签 r_t^{pseudo}
- **在线阶段**：训练轻量位姿头，从当前观测（DINO特征+可选IMU）估计r_t，用离线伪标签监督
- **推理**：只需轻量位姿头（无需DUSt3R在线运行）

**Reasoning**：这解决了弱监督脆弱性问题，且不增加推理计算量。DUSt3R是现成工具，不作为论文贡献。

---

### 4. IMPORTANT: 收窄Claim 2，聚焦表示迁移而非零样本导航

**Reviewer说**："零样本迁移到四足数据集"混淆了太多变量。

**Action**：将Claim 2从"零样本完整导航迁移"收窄为：
- **Claim 2A**：冻结视角感知潜空间动力学，仅重训规划器/控制器适配层，验证视角建模的跨平台可复用性
- **Claim 2B**（备选）：在公开四足数据集上评估世界模型的转弯预测误差，而非完整导航成功率

这保留了通用性故事，同时让实验结论更清晰可归因。

---

## Revised Proposal (Full)

# Research Proposal: Explicit Viewpoint State for Motion-Coupled Visual Navigation (EV-WM)

## Problem Anchor
- **Bottom-line problem**: 移动机器人（球型/四足/无人机）在导航中运动与视觉观测强耦合——体态变化持续改变摄像头视角，导致现有潜空间视觉世界模型无法准确预测"执行某动作后会看到什么"，ImageGoal导航规划失效。
- **Must-solve bottleneck**: DINO-WM类模型将动作映射到场景位移，忽略视角旋转。更根本地：现有模型没有显式的视角状态，导致MPPI rollout时无法预测视角演化，规划在转弯主导的场景系统性失败。
- **Non-goals**: No VLN; no pixel-level generation; no SLAM; no end-to-end pixel→action; no online 3D reconstruction.
- **Constraints**: 双目RGB；机器人GPU显存有限；GPU服务器训练；可采集真实数据；6个月投TRO。
- **Success**: 真实球型机器人ImageGoal导航SR明显优于无视角状态的基线；跨平台表示迁移实验证明通用性。

---

## Technical Gap

**根本问题**：现有潜空间世界模型（DINO-WM、ReL-NWM）没有视角状态的概念：
```
标准模型：(z_t, a_t) → ẑ_{t+1}          ← 视角隐含在z_t中，不显式建模
MPPI规划：展开K步动作序列 → 找最小goal距离
问题：转弯时视角旋转，z_{t+1}的语义分布变化，
      但模型没有机制预测这种变化，导致规划崩溃
```

**为什么简单扩展不够**：
- 直接加Δθ作为条件输入（原方案）：解决了预测端，但rollout时未来Δθ无法预测
- 用像素级模型（NWM/ANWM）：无法在机器人GPU部署
- 数据增强：不能建模物理耦合关系

**最小必要机制**：引入显式视角潜状态r_t，训练联合转移模型同时预测(ẑ_{t+1}, r̂_{t+1})。

---

## Method Thesis
通过在DINO潜空间中引入显式视角状态r_t和pose-conditioned ray token interaction，构建能够准确预测视角演化的联合视角-场景转移模型，实现转弯主导场景下的鲁棒ImageGoal导航规划。

---

## Contribution Focus
- **Dominant**: 显式视角潜状态世界模型（EV-WM）——联合建模场景和视角演化的潜空间转移模型，用ray-conditioned token interaction建模视角变换物理
- **Supporting**: 离线几何教师蒸馏的轻量视角状态估计——用DUSt3R等基础模型产生高质量伪标签，蒸馏到轻量在线位姿头，无需推理时运行几何模型
- **Non-contributions**: 不提出新规划算法；不提出新特征编码器；不做像素生成

---

## Proposed Method

### Complexity Budget
- **Frozen/reused**: DINOv2编码器；MPPI规划器；现有PPO底层控制器；DUSt3R（离线，不在线运行）
- **New trainable**:
  1. EV-ViT Predictor：在ViT预测器中加入ray-conditioned cross-attention，联合输出(ẑ_{t+1}, r̂_{t+1})
  2. 轻量视角状态估计头：从DINO特征(+IMU)估计r_t，由离线伪标签监督
- **Excluded**: 像素解码器；在线3D重建；完整SO(3)建模（只建模平台主要DoF）

### System Overview

```
双目RGB图像 (左/右)
    |
    v
[冻结DINOv2] → patch特征 z_t ∈ R^{N×E}
    |
    +→ [轻量视角状态估计头] → r_t ∈ R^d（当前视角状态）
           (DINO特征 + 可选IMU)
           (离线DUSt3R伪标签监督)
    |
    v
[EV-ViT Predictor]
输入：z_{t-H:t}, r_{t-H:t}, a_{t-H:t}
核心：ray-conditioned token interaction
      token_i' = concat(z_i, ray_i(r_t))
      跨时间步的cross-attention用旋转ray索引
输出：(ẑ_{t+1}, r̂_{t+1})  ← 联合预测场景特征和视角状态
    |
    v
[MPPI规划器]
候选动作序列 → K步联合rollout (ẑ, r̂)_K
→ 比较 ẑ_K 与目标图像特征 → 选最优动作序列
    |
    v
[PPO底层控制器] → 电机指令 @ 50Hz
```

### Core Mechanism：EV-ViT Predictor

**视角状态 r_t**：
- 紧凑向量 r_t ∈ R^d（d=8~16），编码当前相机朝向
- 对球型机器人：主要建模偏航（yaw）+ 滚转诱导的pitch分量
- 不强制完整SO(3)，只建模平台主要耦合自由度

**Ray-conditioned Token Interaction**：
```
# 每个patch token附加相机射线嵌入
ray_i(r_t) = R(r_t) · base_ray_i    # 用r_t旋转基准射线
token_i' = Linear([z_i, ray_i(r_t)])  # 几何感知token

# EV-ViT内部的跨时间步cross-attention
# Query来自当前时刻token（带当前ray）
# Key/Value来自历史时刻token（带历史ray）
# 自然实现视角旋转后的token对应关系
```

**联合训练目标**：
```
L_total = L_scene + λ₁·L_vp + λ₂·L_ray

L_scene = cosine_loss(ẑ_{t+1}, z_{t+1})    # 场景特征预测（cosine，比L2更适合DINO）
L_vp    = ||r̂_{t+1} - r_{t+1}^{pseudo}||²   # 视角状态预测
L_ray   = alignment_loss(ray_{predicted}, stereo_geometry)  # 几何一致性
```

**离线伪标签生成**：
- 用DUSt3R对采集数据做离线处理，产生高质量相机位姿序列
- 从位姿序列提取r_t时序标签
- 训练轻量视角头：(z_t, [IMU]) → r_t（在线估计）

### Training Plan

**数据采集**（真实机器人）：
- 500-1000条轨迹（左目+右目+IMU+电机指令）
- 覆盖直行/转弯/不平整地形等多样运动模式
- 采集后做一次离线DUSt3R处理产生位姿伪标签（约1天GPU计算）

**训练阶段**：
- Stage 1：离线DUSt3R → 位姿伪标签 r_t^{pseudo}
- Stage 2：训练轻量视角头（冻结DINOv2，监督学习）
- Stage 3：联合训练EV-ViT（冻结DINOv2+视角头，训练转移模型）
- Stage 4：MPPI超参调整

**训练量**：~22M参数EV-ViT（ViT-S规模），约8-12h on GPU server

---

## Claim-Driven Validation

### Claim 1（主）：EV-WM在转弯主导场景下显著提升ImageGoal SR

- **Minimal experiment**: 真实球型机器人，EV-WM vs DINO-WM vs ReL-NWM vs ANWM
- **关键对照**: 高转弯场景（需多次转弯到达目标）vs 低转弯场景（基本直行）
  → 预期：高转弯场景差距>20%，低转弯场景差距<5%（证明对症）
- **Metric**: SR（主），场景特征预测MSE（辅），推理延迟（辅）

### Claim 2（支撑）：视角感知潜状态可跨平台复用

- **Experiment**: 冻结EV-WM的视角-场景转移模型，仅重训规划适配层，在公开四足数据集（HuRoN/ReCon）上评估转弯预测误差
- **Metric**: 特征预测误差（转弯 vs 直行），而非完整导航SR（避免controller mismatch混淆）

### Ablations

| 消融项 | 验证的必要性 |
|--------|------------|
| 无显式视角状态 → 退化为DINO-WM | 视角状态的必要性 |
| 无ray-conditioned interaction → 退化为bias | ray条件化的必要性 |
| 无离线DUSt3R伪标签 → 只用stereo几何 | 高质量伪标签的价值 |
| 完整SO(3) vs 最小DoF | 建模DoF选择的合理性 |

---

## Compute & Timeline
- GPU-hours：~80h（数据采集、DUSt3R离线、模型训练、实验评测）
- Timeline：月1数据+DUSt3R处理，月2训练，月3真实机器人，月4对比消融，月5-6写作投稿
