# Round 2 Refinement

## Problem Anchor（逐字不变）
- **Bottom-line**: 移动机器人（球型/四足/无人机）运动与视觉观测强耦合——体态变化持续改变摄像头视角，导致现有潜空间视觉世界模型无法准确预测"执行某动作后会看到什么"，ImageGoal导航规划失效。
- **Must-solve bottleneck**: DINO-WM类模型没有显式视角状态，MPPI rollout无法传播视角演化，在转弯主导场景系统性失败。
- **Non-goals**: No VLN; no pixel-level generation; no SLAM; no online 3D reconstruction.
- **Constraints**: 双目RGB；机器人GPU显存有限；GPU服务器训练；可采集真实数据；6个月投TRO。
- **Success**: 真实球型机器人SR明显优于无视角状态基线；跨平台表示迁移实验证明通用性。

---

## Anchor Check
- 原始瓶颈：视角状态缺失导致rollout失效 ✓ 未变
- 修订后方法仍解决该问题 ✓
- 拒绝的建议：不扩大Claim 2，不加复杂transfer协议

## Simplicity Check
- 主贡献：显式视角潜状态 r_t + 多步viewpoint-consistent rollout
- 修订只补充两处缺失定义，不增加新模块
- ray_alignment_loss如无精确定义则删除（reviewer建议，接受）

---

## Changes Made

### 1. BLOCKING FIX：加入多步latent overshooting训练

**Reviewer说**：训练是one-step损失，规划用multi-step rollout，存在train-test mismatch。

**Action**：在Stage 2训练中加入**多步overshooting目标**：

```
# 标准one-step（保留）
L_1step = cosine(ẑ_{t+1}, z_{t+1}) + λ·||r̂_{t+1} - r_{t+1}^*||²

# 新增：K步rollout一致性损失（K=3,5）
# 从时刻t出发，用预测的r̂_{t+1}（而非teacher r_t+1）展开K步
r̂_{t+1} = r_head(z_t, a_t)        ← 用预测值，不用teacher
ẑ_{t+2} = EV-ViT(ẑ_{t+1}, r̂_{t+1}, a_{t+1})
...

L_Kstep = Σ_{k=1}^{K} γ^k · cosine(ẑ_{t+k}, z_{t+k})

L_total = L_1step + λ_K · L_Kstep
```

**实现细节**：
- K=3开始，后期可增到5
- γ=0.9衰减权重（避免远步误差主导）
- 训练时交替：50% one-step样本 + 50% K-step样本（节省计算）

**Reasoning**：这直接对齐训练目标和推理行为，是latent WM中标准的"overshooting"技术（DreamerV2/V3也用）。参数增量为零，只是换训练方式。

---

### 2. BLOCKING FIX：精确定义 r_t 的坐标系与初始化

**Reviewer说**：r_t相对哪个参考系？如何初始化/重置？DUSt3R伪标签如何与IMU对齐？

**Action**：明确定义如下：

**坐标系选择：相机帧相对于episode起始帧的相对旋转**

```
r_t = 紧凑表示当前时刻相机朝向相对于episode起始时刻的旋转
    = R_{cam,t}^{cam,0} ∈ 表示为 sin-cos对：[sin(ψ), cos(ψ), sin(φ), cos(φ)]
                                              偏航ψ       俯仰φ

其中：
- 参考系：episode起始帧的相机坐标系（r_0 = 0，天然初始化）
- 维度：d=4（sin-cos编码，避免角度周期性问题）
- 球型机器人：只建模yaw(ψ) + pitch(φ)（滚转roll对导航影响小，可选）
```

**DUSt3R伪标签对齐**：
```
DUSt3R输出：世界坐标系下的相机位姿序列 T_t ∈ SE(3)
转换：r_t = extract_yaw_pitch(T_t^{-1} · T_0)  ← 相对第0帧
           转换为sin-cos表示
验证：与IMU积分对比，过滤RMSE > 阈值的轨迹段（置信度控制）
```

**损失函数改为角度对应损失**：
```
# 替换原L2 → geodesic/sin-cos loss
L_vp = ||sin(ψ̂) - sin(ψ*)||² + ||cos(ψ̂) - cos(ψ*)||²
     + ||sin(φ̂) - sin(φ*)||² + ||cos(φ̂) - cos(φ*)||²
```

**Reasoning**：相对起始帧的定义天然提供初始化（r_0=0），避免累积漂移，且与DUSt3R输出的对齐方式清晰。sin-cos编码是角度回归的标准做法。

---

### 3. 接受：删除 ray_alignment_loss

**Reviewer说**：ray_alignment_loss定义不清晰，可能读作额外正则项。

**Action**：删除 L_ray，只保留 L_scene + L_vp + L_Kstep。

Ray embedding的几何正确性由r_t伪标签监督隐式保证（r_t准确→ray方向准确），无需单独损失。

---

### 4. 接受：伪标签置信度加权

**Reviewer说**：不应平等对待所有DUSt3R伪标签，低质量场景会引入噪声。

**Action**：
- DUSt3R输出重建置信度图，提取每段轨迹的平均置信度c_t
- 加权训练样本：损失乘以 w_t = σ(c_t - threshold)
- 低纹理/运动模糊段自动降权，不需要手动筛选

---

## Revised Proposal (Full — Round 2)

# Research Proposal: EV-WM — Explicit Viewpoint State for Motion-Coupled Visual Navigation

## Problem Anchor
（同上，不变）

## Technical Gap
（同Round 1修订版，不变）

## Method (EV-WM)

### Viewpoint State Definition
```
r_t ∈ R⁴：当前相机朝向相对episode起始帧的旋转
         = [sin(ψ_t), cos(ψ_t), sin(φ_t), cos(φ_t)]
         ψ：yaw（偏航），φ：pitch（俯仰）
         r_0 = [0, 1, 0, 1]（起始帧，天然初始化）
```

### 系统管线（更新）
```
双目RGB → [冻结DINOv2] → z_t ∈ R^{N×E}
[z_t, IMU_t] → [轻量位姿头] → r_t ∈ R⁴
              （DUSt3R置信度加权伪标签监督）

ray_i(r_t) = R(r_t) · base_ray_i  ← 用r_t旋转第i个patch基准射线
token_i' = Linear([z_i, ray_i(r_t)])

[token'_{t-H:t}, r_{t-H:t}, a_{t-H:t}] → [EV-ViT] → (ẑ_{t+1}, r̂_{t+1})

MPPI rollout（K步，viewpoint-consistent）：
  step k: ray_i(r̂_k) → token'_k → EV-ViT → (ẑ_{k+1}, r̂_{k+1})
  用预测的r̂，不用teacher → 真正的closed-loop rollout
  最终：compare ẑ_K with goal features → optimal action
```

### 训练目标（更新）
```
L_scene = cosine_loss(ẑ_{t+1}, z_{t+1})           ← 场景特征
L_vp    = geodesic_loss(r̂_{t+1}, r_{t+1}^*)       ← sin-cos角度损失
L_Kstep = Σ_{k=1}^{K} γ^k cosine(ẑ_{t+k}, z_{t+k})  ← 多步overshooting

L_total = L_scene + λ₁·L_vp + λ₂·L_Kstep

权重：w_t = σ(DUSt3R置信度 - threshold)  ← 伪标签置信度加权
```

### 训练阶段（更新）
```
Stage 0：DUSt3R离线处理所有轨迹 → 位姿伪标签 + 置信度
Stage 1：训练轻量位姿头（冻结DINOv2，置信度加权监督）
Stage 2：联合训练EV-ViT + 位姿头
         - 50% one-step样本（L_scene + L_vp）
         - 50% K-step样本（L_scene + L_vp + L_Kstep，K=3）
Stage 3：MPPI超参调整（冻结所有模型）
```

## Validation（保持Round 1修订版，微调表述）

### Claim 1：EV-WM显著提升转弯主导场景的ImageGoal导航性能
- 实验：真实球型机器人，高转弯vs低转弯路线，对比DINO-WM/ReL-NWM/ANWM
- 关键消融：A1（r_t=0降级为DINO-WM）；A2（positional bias vs ray interaction）；A3（1-step vs K-step训练）；A4（无DUSt3R置信度加权）

### Claim 2：视角感知转移模型可跨平台迁移（表示迁移，非完整零样本导航）
- 实验：冻结EV-WM转移模型，仅训练小型相机/动作适配头，在四足数据集(HuRoN)上评估转弯预测误差
- 安全声明措辞："在固定视角感知潜空间下，跨平台的转弯诱导特征变化预测误差显著低于DINO-WM基线"

## 新颖性措辞（保守化）
"首个在轻量潜空间世界模型中引入显式视角状态并支持多步viewpoint-consistent MPPI rollout的方法，在真实转弯主导机器人导航场景中验证"
（不再使用"FIRST ever in any latent WM"的表述）
