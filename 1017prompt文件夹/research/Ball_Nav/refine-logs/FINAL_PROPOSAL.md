# EV-WM: Explicit Viewpoint State Latent World Model for Motion-Coupled Visual Navigation

**Final Proposal — 9.1/10, READY for TRO**

---

## Problem Anchor

- **Bottom-line**: 移动机器人（球型/四足/无人机）在导航中运动与视觉观测强耦合——体态变化持续改变摄像头视角，导致现有潜空间视觉世界模型无法准确预测"执行某动作后会看到什么"，ImageGoal导航规划失效。
- **Must-solve bottleneck**: DINO-WM类模型没有显式视角状态，MPPI rollout无法传播视角演化，在转弯主导场景系统性失败。
- **Non-goals**: No VLN; no pixel-level generation; no SLAM; no online 3D reconstruction.
- **Constraints**: 双目RGB；机器人GPU显存有限；GPU服务器训练；可采集真实数据；6个月投TRO。
- **Success**: 真实球型机器人SR明显优于无视角状态基线；跨平台表示迁移实验证明通用性。

---

## Technical Gap

**根本问题**：现有潜空间世界模型 (z_t, a_t) → ẑ_{t+1} 将视角变化隐含在特征中，无法在MPPI rollout中显式传播视角演化。球型机器人转弯时，相机同步旋转，同一场景在转弯前后的DINO特征分布完全不同，但模型没有任何机制预测这种变化，导致K步rollout中预测误差急剧累积。

**为什么简单扩展不够**：
- 直接加Δθ作为条件输入：rollout时未来Δθ无法预测（原Round 1问题）
- 像素级世界模型（NWM/ANWM）：机器人GPU无法部署
- 数据增强：不能建模物理耦合关系

**最小必要机制**：引入显式视角潜状态 r_t，训练联合转移 (z_t, r_t, a_t) → (ẑ_{t+1}, r̂_{t+1})，配合多步overshooting训练实现closed-loop rollout一致性。

---

## Method Thesis

通过在DINO潜空间中引入显式视角状态 r_t 和 pose-conditioned ray token interaction，构建能够准确预测视角演化的联合视角-场景转移模型，并通过多步closed-loop overshooting训练使世界模型的训练目标与MPPI推理行为对齐，实现转弯主导场景下的鲁棒ImageGoal导航规划。

---

## Contribution Focus

- **Dominant**: EV-WM — 显式视角潜状态 + 多步viewpoint-consistent rollout的轻量潜空间世界模型
- **Supporting**: 离线DUSt3R教师蒸馏（置信度加权）→ 轻量在线位姿头，无推理时开销
- **Non-contributions**: 不提出新规划算法（复用MPPI）；不提出新特征编码器（复用DINOv2）；不做像素生成

---

## Proposed Method

### 视角状态定义

```
r_t ∈ R⁴ = [sin(ψ_t), cos(ψ_t), sin(φ_t), cos(φ_t)]

ψ：偏航（yaw），φ：俯仰（pitch）
参考系：相对episode起始帧的相机坐标系（世界坐标→起始相机坐标）
初始化：r_0 = [0, 1, 0, 1]（天然零初始化）

DUSt3R伪标签提取：
  R_rel = T_0^{-1} · T_t  （T = world-to-camera变换）
  提取 (ψ_t, φ_t)，转为sin-cos表示
  置信度加权：w_t = σ(DUSt3R置信度 - threshold)
  IMU一致性过滤：丢弃 |r_t^{DUSt3R} - r_t^{IMU}| > ε 的段
```

### 系统管线

```
双目RGB (左/右)
    │
    ▼
[冻结DINOv2编码器]
    │
    ├──► patch特征 z_t ∈ R^{N×E}
    │
    └──► [轻量位姿头（z_t + IMU）]
              │  DUSt3R置信度加权监督
              ▼
           r_t ∈ R⁴（当前视角状态）

ray_i(r_t) = R(r_t) · base_ray_i     ← 旋转第i个patch基准射线
token_i'   = Linear([z_i, ray_i(r_t)])  ← ray-augmented token

[token'_{t-H:t}, r_{t-H:t}, a_{t-H:t}]
    │
    ▼
[EV-ViT Predictor]
    │  ray-conditioned cross-attention across timesteps
    ▼
(ẑ_{t+1}, r̂_{t+1})  ← 联合预测

MPPI Rollout（K步，closed-loop）：
  每步 k：用预测的 r̂_k 计算 ray_i(r̂_k) → token'_k → EV-ViT → (ẑ_{k+1}, r̂_{k+1})
  用预测值，不用teacher → 真正的视角感知闭环规划
  最终：compare ẑ_K 与目标图像特征 → 选最优动作序列

[PPO底层控制器（已有）] → 电机指令 @ 50Hz
```

### 核心机制：Ray-Conditioned Token Interaction

每个DINO patch token i附加该patch对应的相机射线方向（由当前视角状态r_t旋转）：
- 当机器人转向时，r_t更新，ray_i(r_t)随之旋转
- EV-ViT的跨时间步cross-attention利用ray-augmented token，自然建模"哪个patch看向场景哪个方向"
- 视角旋转 = ray embedding旋转，不需要显式像素投影

### 训练目标

```
L_scene = cosine_loss(ẑ_{t+1}, z_{t+1})                      ← DINO特征余弦损失

L_vp = Σ_{d∈{ψ,φ}} [||sin(d̂)-sin(d*)||² + ||cos(d̂)-cos(d*)||²]  ← sin-cos角度损失

L_Kstep = Σ_{k=1}^{K=3} γ^k · cosine(ẑ_{t+k}, z_{t+k})     ← 多步overshooting
           γ = 0.9，closed-loop：每步用 r̂_k 不用teacher

L_total = L_scene + λ₁·L_vp + λ₂·L_Kstep
          以 w_t（DUSt3R置信度）加权所有样本
```

### 训练阶段

```
Stage 0（离线）：
  DUSt3R处理所有采集轨迹 → 位姿伪标签 {r_t*} + 置信度分数
  应用IMU一致性过滤，生成置信度权重 {w_t}

Stage 1（~4h GPU）：
  训练轻量位姿头：(z_t, IMU_t) → r_t
  损失：L_vp，置信度加权

Stage 2（~8-12h GPU）：
  联合训练EV-ViT + 位姿头（DINOv2冻结）
  50% one-step样本（L_scene + L_vp）
  50% K-step closed-loop样本（L_scene + L_vp + L_Kstep）

Stage 3（~2h）：
  MPPI超参调整（冻结所有模型）
```

### 复杂度预算

| 组件 | 状态 | 参数量 |
|------|------|--------|
| DINOv2编码器 | 冻结复用 | ~86M（不训练）|
| EV-ViT Predictor | 新训练 | ~23M（ViT-S + ray头）|
| 轻量位姿头 | 新训练 | ~0.5M |
| MPPI规划器 | 复用 | 0 |
| PPO控制器 | 复用 | 已有 |
| DUSt3R | 仅离线 | 不在推理中 |

---

## Claim-Driven Validation

### Claim 1（主）：EV-WM显著提升转弯主导场景的ImageGoal导航成功率

- **实验设置**：真实球型机器人，室内/室外混合场景
- **路线设计**：高转弯组（总转向>90°）vs 低转弯组（基本直行）
- **对比方法**：DINO-WM / ReL-NWM / ANWM（参考基线）
- **指标**：SR（主），Feature MSE by turning intensity（辅），推理延迟（辅）
- **关键预期**：高转弯组SR差距显著，低转弯组差距小（证明对症而非一般性提升）

### Claim 2（支撑）：视角感知转移模型可跨平台迁移

- **实验**：冻结EV-WM转移模型，训练小型相机/动作适配头，在HuRoN四足数据集评估转弯预测误差
- **安全声明**："冻结视角感知潜动力学下，跨平台转弯诱导特征变化预测误差显著低于DINO-WM基线"
- **不声明**：完整零样本导航迁移

### 消融实验

| 消融 | 验证 |
|------|------|
| A1: r_t=0 → DINO-WM | 显式视角状态的必要性 |
| A2: positional bias → ray interaction | ray条件化的表达能力 |
| A3: 1-step训练 → K-step overshooting | 多步训练的必要性 |
| A4: 无DUSt3R置信度加权 | 伪标签质量控制的价值 |

### 诊断实验（非主要claim，增强可信度）

- K步rollout中视角状态漂移量 r̂_K vs 真实r_K（IMU/DUSt3R）
- 保留数据统计：按运动类型（转弯/直行）报告过滤前后样本比例
- DUSt3R置信度分布（避免reviewer质疑选择性删除困难样本）

---

## 新颖性声明（保守）

> "首个在轻量潜空间世界模型中引入显式视角状态并支持多步viewpoint-consistent MPPI rollout的方法，在真实转弯主导机器人导航场景中验证，且无需在线运行3D重建或像素级生成。"

---

## 时间规划

| 月份 | 任务 |
|------|------|
| 月1 | 数据采集（500-1000条轨迹）+ DUSt3R离线处理 |
| 月2 | Stage1-2模型训练 + 仿真初步验证 |
| 月3 | 真实球型机器人实验（Claim 1主实验）|
| 月4 | 对比实验 + 消融 + 诊断实验 + HuRoN迁移验证 |
| 月5 | 论文写作 |
| 月6 | 修改投稿 |

**总GPU用量估计**：~80-100 GPU-hours（训练 + 实验评测）
