# 球形机器人联合世界模型导航：研究 Pipeline 与阶段计划

## 0. 文档定位

本文档回答“具体按什么顺序开展研究”。方法定义与研究动机以同目录下的 `POINTGOAL_STRUCTURED_WORLD_MODEL_PROPOSAL.md` 为准。

最终交付不是相互独立的动力学模型和视觉模型，而是一个统一的：

> **双目视觉—本体动力学—空间占据联合世界模型**

阶段拆分只用于验证数据、监督和各模块是否可靠，并为联合训练提供稳定初始化。最终论文必须以联合模型为主方法，并用独立串联模型作为对照。

本文默认研究范围如下：

- 任务：静态室内环境 PointGoal 导航；
- 输入：同步双目 RGB、本体状态历史、动作历史、当前相对目标坐标；
- 世界模型输出：未来机器人状态、未来视觉 latent、当前与未来局部 BEV 占据图；
- 环境类别：free / occupied / unknown；
- 全局规划：Hybrid A* 或 State Lattice；
- 局部规划：iCEM；
- 视觉基座：LeWorldModel（LeWM）的端到端 JEPA 与 SIGReg；
- 不预测动态障碍，不生成未来 RGB，不学习独立相机位姿。

---

## 1. 核心科学问题与论文主张

### 1.1 核心科学问题

在解析动力学难以准确建模的球形机器人上，联合预测“机器人实际会怎样运动”和“运动后周围障碍在哪里”，是否比以下方法更适合真实闭环导航：

1. 解析或纯状态动力学模型；
2. 只预测视觉 latent 的 LeWM；
3. 动力学模型与视觉模型独立串联；
4. 当前 occupancy 加几何 Warp；
5. 不包含未来空间监督的联合模型。

### 1.2 预注册式研究假设

在正式主实验前冻结以下假设，避免根据结果事后修改论文叙事。

**H1：联合建模假设**

联合 Transformer 比参数量相当的独立串联模型具有更低的多步状态误差、future-BEV 误差和闭环碰撞率。

**H2：空间保持假设**

对当前和未来 latent 使用共享 BEV 空间头监督，比只监督当前 latent 更能保持多步 rollout 中的障碍信息。

**H3：真实动力学假设**

利用真实交互数据训练的历史条件动力学表示，比解析模型和仅仿真辨识模型具有更低的实物多步运动误差。

**H4：导航转化假设**

模型预测指标的改善能够转化为 PointGoal 成功率提高和碰撞率下降，而不只是离线误差降低。

### 1.3 论文贡献边界

本文不把以下内容声明为原创：

- LeWM 的 SIGReg 防坍塌机制；
- JEPA 的 next-embedding prediction；
- BEV occupancy 本身；
- Hybrid A*、State Lattice 或 iCEM；
- 相机固定外参和几何坐标变换。

主要创新主张应聚焦：

1. 面向球形机器人难建模动力学的视觉—本体联合 latent transition；
2. 以预测实际运动条件化未来环境 latent；
3. 训练与规划共用的共享 BEV 空间头；
4. 联合模型相对独立串联模型和几何 Warp 的真实导航收益。

---

## 2. 最终联合世界模型

### 2.1 输入与输出

给定历史窗口长度 $H$ 和预测长度 $K$：

\[
\mathcal I_t=
\left\lbrace
o_{t-H:t}^{L},
o_{t-H:t}^{R},
x_{t-H:t},
a_{t-H:t-1}
\right\rbrace.
\]

其中：

- $o_t^L,o_t^R$：同步左右目 RGB；
- $x_t$：机器人动力学状态；
- $a_t$：发送到执行器且与传感器对齐的动作；
- PointGoal $g_t$ 只进入规划代价，不作为世界动力学的必要输入。

世界模型输出：

\[
\hat{\mathcal Y}_{t+1:t+K}
=
\left\lbrace
\hat x_{t+1:t+K},
\hat z_{t+1:t+K},
\hat O_{t:t+K}
\right\rbrace.
\]

### 2.2 模块化实现、整体式建模

```text
左右目图像序列
      │
      ▼
双目 LeWM 编码器 E_stereo ─────────► 视觉空间 tokens z_t
      │                                      │
      └────► 共享 BEV 头 D_spatial ────────► 当前占据 O_t
                                             │
本体状态历史 ──► 状态编码器 E_x ──► 状态 tokens
动作历史/候选动作 ─► 动作编码器 E_a ─► 动作 tokens
                                             │
             视觉 + 状态 + 动作 tokens ──────┘
                              │
                              ▼
                   因果联合 Transformer T_joint
                              │
                 ┌────────────┴────────────┐
                 ▼                         ▼
          动力学输出头 D_x           未来 latent 头 D_z
                 │                         │
                 ▼                         ▼
        未来状态 x_hat              未来 latent z_hat
                                           │
                                           ▼
                              共享 BEV 头 D_spatial
                                           │
                                           ▼
                                  未来占据 O_hat
```

所有模块属于同一个世界模型 $F_\theta$。视觉、状态和动作采用不同输入编码器，是为了保持模态接口清晰；它们在共享的因果 Transformer 中交互，并由同一个训练目标联合优化。

### 2.3 联合转移

\[
e_t^z=E_{\mathrm{stereo}}(o_t^L,o_t^R),
\quad
e_t^x=E_x(x_t),
\quad
e_t^a=E_a(a_t),
\]

\[
h_{t+1}=T_{\mathrm{joint}}
\left([e_{t-H:t}^z;e_{t-H:t}^x;e_{t-H:t}^a]\right),
\]

\[
\hat x_{t+1}=D_x(h_{t+1}),
\qquad
\hat z_{t+1}=D_z(h_{t+1}),
\]

\[
\hat O_t=D_{\mathrm{spatial}}(z_t),
\qquad
\hat O_{t+1}=D_{\mathrm{spatial}}(\hat z_{t+1}).
\]

空间头对当前和未来 latent 共享参数。若最终采用几何 Warp，它只作为联合 predictor 内的确定性先验或残差基线，不取代联合模型。

### 2.4 总训练目标

\[
\mathcal L=
\mathcal L_{\mathrm{pred}}
+\lambda_{\mathrm{sig}}\mathcal L_{\mathrm{SIGReg}}
+\lambda_x\mathcal L_{\mathrm{dyn}}
+\lambda_O^{\mathrm{obs}}\mathcal L_{\mathrm{BEV}}^{\mathrm{obs}}
+\lambda_O^{\mathrm{future}}\mathcal L_{\mathrm{BEV}}^{\mathrm{future}}
+\lambda_K\mathcal L_{\mathrm{rollout}}.
\]

训练分阶段启用损失项，但最终联合微调必须同时优化完整目标。

---

## 3. 数据契约与坐标系统

### 3.1 每个时间步必须保存的数据

| 字段 | 说明 | 训练用途 |
|---|---|---|
| `timestamp` | 统一单调时间戳 | 多模态同步 |
| `left_rgb`, `right_rgb` | 同步双目图像 | LeWM 编码器 |
| `base_pose` | 世界系下 base link 位姿 | 状态增量和标签生成 |
| `lin_vel`, `ang_vel` | 机体速度 | 动力学状态 |
| `joint_pos`, `joint_vel` | 内部机构状态 | 动力学状态 |
| `commanded_action` | 控制器发出的动作 | 行为条件 |
| `applied_action` | 实际执行或反馈动作 | 时延与执行器建模 |
| `collision` | 碰撞标志 | 导航评估与安全标签 |
| `depth/pointcloud` | 训练期空间教师 | BEV 标签生成 |
| `camera_intrinsics` | 左右目内参 | 深度反投影 |
| `stereo_extrinsics` | 双目外参 | 双目几何 |
| `base_to_camera` | 固定外参 | base 系 BEV 标签 |

### 3.2 坐标系约定

在写训练代码前固定并测试：

- 世界坐标系：只用于目标、轨迹真值和评估；
- base link 坐标系：动力学状态和局部 PointGoal；
- camera 坐标系：图像、深度和点云；
- BEV 坐标系：与 base link 对齐，以机器人当前位置为原点；
- 相机位姿由 base pose 与固定外参派生，不作为学习状态。

### 3.3 BEV 标签生成

推荐初始规格：

- 范围：$8\,\mathrm m\times8\,\mathrm m$；
- 分辨率：$0.1\,\mathrm m$；
- 网格：$80\times80$；
- 类别：free / occupied / unknown。

生成步骤：

1. 深度反投影为相机点云；
2. 使用固定外参变换到 base link；
3. 删除地面点及机器人自身点；
4. 根据机器人可碰撞高度范围筛选障碍点；
5. 射线经过网格标记 free；
6. 射线终点网格标记 occupied；
7. 未被有效射线覆盖的网格标记 unknown；
8. 标签生成阶段不做机器人半径膨胀；膨胀属于规划层。

### 3.4 数据划分原则

禁止按相邻帧随机划分，否则近重复图像会造成泄漏。应按以下层级划分：

1. 场景级：训练、验证、未见测试场景互斥；
2. 轨迹级：同一 episode 只属于一个 split；
3. 实物日期级：不同采集日期划分，测量跨日期泛化；
4. 动作分布级：保留一组高转向/急制动轨迹作为分布外测试。

---

## 4. 总体研究流程

```text
Phase 0  冻结任务与接口
   │
Phase 1  数据审计和同步
   │
Phase 2  BEV教师与数据管线
   │
Phase 3  单模块健康检查（非最终模型）
   │
Phase 4  联合单步世界模型
   │
Phase 5  联合多步 rollout + future-BEV
   │
Phase 6  独立串联/几何 Warp/LeWM 基线
   │
Phase 7  PointGoal + iCEM 闭环
   │
Phase 8  仿真主实验与消融
   │
Phase 9  真实数据与实物闭环
   │
Phase 10 统计、图表与论文写作
```

Go/No-Go 数值是第一版工程门槛。完成小规模 pilot 后允许基于噪声水平调整一次，但必须在正式测试前冻结，并在实验记录中保留修改原因。

---

## 5. Phase 0：冻结研究契约

### 目标

消除任务、动作、状态、坐标系和论文主张的歧义。

### 任务

1. 固定 PointGoal 成功定义，例如终点距离和停稳条件；
2. 固定动作接口，确保采集、模型和 iCEM 使用同一变量；
3. 固定状态向量及每个分量的坐标系和单位；
4. 固定历史窗口 $H$、初始 rollout horizon $K$ 和控制频率；
5. 固定 BEV 原点、方向、范围、分辨率和类别；
6. 建立 H1–H4 对应的实验矩阵；
7. 将旧 EV-WM/ImageGoal 文档标记为历史材料，避免实现时混用。

### 交付物

- 一份机器可读数据 schema；
- 坐标变换单元测试样例；
- 状态和动作字段说明；
- 固定的训练/验证/测试 split 清单；
- 主张—实验对应表。

### Go/No-Go

- 任意一个状态或动作字段仍无法明确坐标系、单位或时间对应关系：No-Go；
- 控制器优化变量与数据动作字段不一致：No-Go；
- 所有接口可由一条 episode 完整解析且无人工猜测：Go。

---

## 6. Phase 1：现有数据审计与 Pilot 数据

### 已知现状

当前 `trajectories_dubins_640x480` 可用于验证加载和短期原型，但存在：

- 图像频率低于物理/控制频率；
- 大量相邻保存图像重复；
- 一个 episode 缺少 `states.npz`；
- 图像尺寸和黑帧异常；
- 动作大量集中在上限；
- 缺少停止、后退、急制动和丰富转向；
- 未直接保存深度或 BEV 标签。

### 任务

1. 编写 episode manifest，记录帧数、时间范围、传感器有效率；
2. 检测重复帧、黑帧、丢帧和分辨率异常；
3. 校验 `(state_t, action_t) -> state_{t+1}` 的时间语义；
4. 将视觉帧之间的完整动作窗口保存下来，而不是只取单个动作；
5. 统计速度、角速度、关节状态和动作覆盖率；
6. 采集一批包含加速、制动、连续左右转和随机激励的 pilot 数据；
7. 按场景和 episode 生成无泄漏 split。

### Go/No-Go

- 有效样本同步成功率低于 99%：No-Go；
- 不能确定动作作用于哪一个状态区间：No-Go；
- 训练和验证存在相邻帧或同 episode 泄漏：No-Go；
- pilot 覆盖直行、左右转、加速、制动和静止：Go。

---

## 7. Phase 2：BEV 教师与空间监督管线

### 目标

先验证标签是否正确，再讨论空间编码能力。

### 任务

1. 在仿真器中同步保存 RGB、深度、base pose 和相机参数；
2. 实现深度到 base-frame BEV 的离线投影；
3. 可视化 RGB、深度、点云和 BEV 四者对齐；
4. 构造 free / occupied / unknown 标签；
5. 对门洞、墙角、近距离障碍、视野边界进行人工抽检；
6. 在真实双目数据上实现双目深度或点云教师；
7. 为低置信度深度保留 invalid/unknown，不强制标成 free。

### 必测用例

- 空场景只产生 free 与 unknown；
- 单墙场景障碍位置误差不超过一个网格；
- 坐标旋转 90° 后 BEV 同步旋转；
- 相机外参平移后障碍位置符合解析结果；
- 未观测区域不得被误标为 free。

### Go/No-Go

- 随机抽检 100 帧中存在系统性旋转、镜像或平移错误：No-Go；
- known 区域覆盖率极低，无法支撑训练：调整 BEV 范围后重测；
- 标签通过几何单元测试和人工抽检：Go。

---

## 8. Phase 3：单模块健康检查

本阶段不是最终架构，只用于证明数据和损失可学习。

### 8.1 动力学健康检查

建立以下基线：

1. 常速度/常角速度；
2. 解析或仿真运动模型；
3. 当前状态 MLP；
4. 历史状态 GRU/LSTM；
5. 解析模型 + 学习残差。

测量 1、5、10、20 个控制步的：

- 位置误差；
- 朝向误差；
- 速度和角速度误差；
- 制动距离误差；
- 转弯半径误差。

### 8.2 双目 LeWM 健康检查

1. 首先在小规模数据上复现 next-latent prediction；
2. 监控 prediction loss、SIGReg、每维标准差和协方差；
3. 比较单目与双目编码；
4. 检查 latent effective rank 和样本间距离；
5. 确认 encoder 没有常量输出或仅记忆场景 ID。

### 8.3 当前 BEV 头健康检查

只从当前真实 latent 解码当前 BEV：

\[
\hat O_t=D_{\mathrm{spatial}}(z_t).
\]

初始目标不是追求最终性能，而是验证 latent 和 BEV 标签之间存在稳定映射。

### Go/No-Go

- LeWM latent 出现坍塌迹象：No-Go，不增加联合模块；
- 当前 BEV occupied Recall 明显低于简单双目深度基线：No-Go；
- 历史动力学模型不能优于常速度基线：No-Go；
- 三个健康检查均通过后进入联合训练。

---

## 9. Phase 4：联合单步世界模型

### 目标

将视觉、状态和动作 token 放入同一个因果 Transformer，训练统一的一步转移。

### 任务

1. 实现 `E_stereo`、`E_x`、`E_a` 的 token 接口；
2. 为不同模态加入 type embedding 和时间位置编码；
3. 使用 causal mask 禁止访问未来观测；
4. 从共享 hidden state 输出未来状态和未来视觉 latent；
5. 当前与未来 latent 共用 `D_spatial`；
6. 联合优化 latent、SIGReg、动力学和 BEV 损失；
7. 检查各损失梯度规模，避免空间损失压制 SIGReg 或动力学损失；
8. 做模态遮蔽测试，确认模型确实使用状态、动作和双目视觉。

### 必要对照

- vision + action；
- state + action；
- vision + state + action；
- 独立动力学模型后接视觉 predictor；
- 参数量匹配的联合模型。

### Go/No-Go

- 联合模型只复制某一模态，遮蔽其他模态不影响结果：No-Go；
- 联合训练导致 latent 坍塌或 BEV 显著退化：调整损失权重/梯度后重测；
- 单步状态、latent 和 BEV 同时达到健康检查基线：Go。

---

## 10. Phase 5：联合多步 Rollout 与 Future-BEV

### 目标

使训练分布与规划时递归 rollout 一致。

### 训练课程

1. 从 (K=1) 开始；
2. 稳定后增加到短规划 horizon；
3. 逐渐提高预测输入比例，减少 teacher forcing；
4. 对每个未来 latent 使用同一个空间头；
5. 对较远 horizon 使用折扣 $\gamma^{k-1}$；
6. 最后在完整损失上联合微调。

### 关键诊断

- 状态误差随 horizon 增长；
- latent error 随 horizon 增长；
- future-BEV mIoU 随 horizon 下降；
- occupied Recall 和碰撞漏检率；
- latent effective rank 是否在递归预测中下降；
- 模型是否产生“不可能但低代价”的规划轨迹。

### Go/No-Go

- future-BEV 比几何 Warp 基线差且没有新视野收益：重新评估学习 predictor 的必要性；
- 远期占据漏检率不可接受：缩短规划 horizon 或加入不确定性惩罚；
- 多步模型在目标规划 horizon 内稳定优于单步递归模型：Go。

---

## 11. Phase 6：证明联合模型不是简单组合

### 必须实现的模型矩阵

| ID | 方法 | 视觉预测 | 动力学历史 | 当前BEV | Future-BEV | 联合交互 |
|---|---|---:|---:|---:|---:|---:|
| B0 | 常速度/解析模型 |  | ✓ |  |  |  |
| B1 | 原始 LeWM | ✓ |  |  |  |  |
| B2 | 双目 LeWM | ✓ |  |  |  |  |
| B3 | 当前 occupancy + 几何 Warp |  | ✓ | ✓ | Warp |  |
| B4 | 动力学 + LeWM 独立串联 | ✓ | ✓ | ✓ | ✓ |  |
| A1 | 联合模型，无 Future-BEV 监督 | ✓ | ✓ | ✓ | 输出 | ✓ |
| A2 | 联合模型，独立空间头 | ✓ | ✓ | ✓ | ✓ | ✓ |
| Ours | 联合模型，共享空间头 | ✓ | ✓ | ✓ | ✓ | ✓ |

### 公平性要求

- 相同训练集和 split；
- 相同历史长度和 horizon；
- 相近参数量，或同时报告参数量和 FLOPs；
- 相同训练步数与调参预算；
- 相同规划 rollout 数量；
- 不允许 Ours 使用额外真值输入。

### 决策条件

如果 Ours 不能稳定优于 B4，则“联合建模”不能作为主要创新，应将论文重新定位为系统集成或空间监督研究；不得只隐藏 B4 结果。

---

## 12. Phase 7：PointGoal 闭环规划

### 12.1 全局层

输入全局/累计静态地图与目标坐标，由 Hybrid A* 或 State Lattice 生成局部 waypoint。全局层只解决拓扑路线，不直接产生电机动作。

### 12.2 局部层

iCEM 在联合世界模型中展开候选动作：

\[
J=
\lambda_gJ_{\mathrm{goal}}
+\lambda_oJ_{\mathrm{obstacle}}
+\lambda_fJ_{\mathrm{feasibility}}
+\lambda_sJ_{\mathrm{smooth}}
+\lambda_uJ_{\mathrm{unknown}}.
\]

其中障碍代价直接使用共享空间头输出：

\[
J_{\mathrm{obstacle}}
=\sum_{k=1}^{K}
C_{\mathrm{collision}}
(\hat x_{t+k},\operatorname{Inflate}(\hat O_{t+k})).
\]

### 12.3 闭环顺序

1. 获取最新双目图像和本体状态；
2. 更新当前联合 latent 和 BEV；
3. 全局层给出局部 waypoint；
4. iCEM 采样候选动作序列；
5. 世界模型并行 rollout 状态、latent 和 future-BEV；
6. 计算目标、碰撞、可达性、平滑和 unknown 风险；
7. 执行第一个或前若干动作；
8. 获取新观测并重新规划。

### Go/No-Go

- 单周期规划延迟超过控制周期：降低 token 数、horizon 或样本数；
- 模型规划利用预测漏洞产生不合理动作：增加动作约束和不确定性代价；
- 无障碍 PointGoal 和静态障碍避障分别通过后，才能进行完整场景实验。

---

## 13. Phase 8：仿真主实验

### 场景组

1. 开阔区域目标；
2. 90°/180° 转向；
3. S 形通道；
4. 窄门；
5. U 型障碍；
6. 目标位于侧后方；
7. 高速制动；
8. 未见静态布局；
9. 视觉外观变化但几何相同；
10. 几何变化但纹理相似。

### 指标

**模型指标**

- 多步位置、朝向、速度和角速度误差；
- latent rollout error；
- current/future-BEV mIoU；
- occupied Precision/Recall；
- 碰撞漏检率；
- 表示 effective rank；
- 推理延迟、参数量和显存。

**导航指标**

- Success Rate；
- SPL / SoftSPL；
- 最终目标距离；
- 碰撞率；
- 路径长度；
- 控制平滑度；
- 单周期规划延迟；
- 每周期模型 rollout 数量。

### 统计要求

- 每种场景使用多个随机种子和起终点；
- 报告均值、标准差或置信区间；
- 对主方法和最强基线做配对统计比较；
- 报告失败次数，而不仅报告成功案例视频；
- 所有规划器在相同计算预算下比较。

---

## 14. Phase 9：真实数据与实物闭环

### 14.1 数据采集顺序

1. 架空或安全支撑下的执行器响应；
2. 空旷区域的随机激励；
3. 不同速度下的连续转向；
4. 加速—制动组合；
5. 接近障碍但不碰撞；
6. 狭窄通道和门洞；
7. 模型规划产生的 on-policy 数据。

### 14.2 安全机制

- 独立急停；
- 速度和摆角硬限制；
- 最小障碍距离保护；
- 规划超时立即停车；
- 高不确定性动作不执行；
- 首次 on-policy 采集使用低速和扩大安全裕量。

### 14.3 Sim-to-Real 实验

比较：

1. 仿真训练直接部署；
2. 仿真预训练 + 少量真实数据微调；
3. 真实数据从头训练；
4. 解析/仿真动力学 + 真实残差；
5. 完整真实数据联合模型。

### 14.4 实物主结果

论文必须至少报告：

- 未参与训练的真实布局；
- 多次重复试验；
- 成功率、碰撞率和停靠误差；
- 模型误差与导航失败的对应关系；
- 联合模型与独立串联模型的实物比较；
- 典型成功和失败轨迹。

---

## 15. Phase 10：论文证据链与写作

### 15.1 主张—证据对应

| 主张 | 必要证据 |
|---|---|
| 联合建模优于串联 | Ours vs B4，参数量和预算公平 |
| Future-BEV 保持空间信息 | A1 vs Ours，多步 IoU/Recall 与碰撞率 |
| 真实数据解决动力学偏差 | 仿真模型 vs 真实模型的实物 rollout |
| 空间头真正改善编码器 | 冻结 encoder 的线性 probe + 闭环导航 |
| 改善能转化为导航收益 | 离线误差与 SR/碰撞率相关分析 |

### 15.2 建议论文图表

1. 联合模型总架构；
2. 数据与 BEV 标签生成流程；
3. 预测状态轨迹与真实轨迹；
4. current/future-BEV 可视化；
5. horizon—误差曲线；
6. 联合模型与串联模型比较；
7. 仿真和实物导航成功率；
8. 真实失败案例分类；
9. 规划计算预算与性能曲线；
10. sim-to-real gap 与真实数据规模曲线。

### 15.3 论文写作顺序

1. 先完成实验表格和失败分析；
2. 再冻结贡献声明；
3. 根据最强结果选择主线，而不是先写夸大的“首个”；
4. 写方法与复现实验细节；
5. 最后写摘要和标题。

---

## 16. 推荐工作包与代码边界

以下是建议边界，不代表当前代码已经实现：

```text
research/ball_wm/
├── configs/
│   ├── data.yaml
│   ├── model.yaml
│   ├── train.yaml
│   └── planner.yaml
├── datasets/
│   ├── trajectory_dataset.py
│   ├── synchronization.py
│   └── bev_labeler.py
├── models/
│   ├── stereo_encoder.py
│   ├── state_encoder.py
│   ├── action_encoder.py
│   ├── joint_world_model.py
│   ├── spatial_head.py
│   └── losses.py
├── planning/
│   ├── global_planner.py
│   ├── icem.py
│   └── costs.py
├── evaluation/
│   ├── dynamics_metrics.py
│   ├── bev_metrics.py
│   └── navigation_metrics.py
└── scripts/
    ├── collect_data.py
    ├── build_manifest.py
    ├── generate_bev_labels.py
    ├── train_world_model.py
    ├── evaluate_world_model.py
    └── run_navigation.py
```

边界原则：

- 数据同步和 BEV 标签生成不放进模型文件；
- `joint_world_model.py` 是最终统一入口；
- 独立动力学和原始 LeWM 只作为 baseline 配置，不成为最终运行管线；
- 规划器只消费世界模型公开输出，不读取模型内部 token；
- 每个指标使用独立评估模块，避免训练代码自行定义有利指标。

---

## 17. 时间与资源安排

以一名主要研究者为参考，使用相对工作周而非固定日期：

| 阶段 | 建议时间 | 主要输出 |
|---|---:|---|
| Phase 0–1 | 2–3周 | 数据契约、审计、pilot 数据 |
| Phase 2 | 2–3周 | BEV 标签管线 |
| Phase 3 | 3–4周 | 动力学、LeWM、当前BEV健康检查 |
| Phase 4 | 3–4周 | 联合单步模型 |
| Phase 5–6 | 4–6周 | 多步模型和关键基线 |
| Phase 7 | 3–4周 | 闭环规划 |
| Phase 8 | 3–4周 | 仿真主实验 |
| Phase 9 | 4–8周 | 实物数据与真实实验 |
| Phase 10 | 3–5周 | 统计、图表和论文 |

预计总周期约 27–41 个有效工作周。实物数据采集、硬件调试和返工通常是最大不确定项，不应按纯训练时间估算。

### 算力优先级

1. 小分辨率、小模型完成数据和损失验证；
2. 先完成关键消融，再扩大模型；
3. 优先增加动作/场景覆盖，而不是只增加重复帧；
4. 规划比较固定 rollout 预算；
5. 只有联合模型明确优于基线后再进行大规模训练。

---

## 18. 停止条件与研究降级路线

### 情况 A：端到端 LeWM 难以稳定训练

降级为冻结或半冻结视觉编码器，但必须将论文主张从“端到端空间表示学习”调整为“动力学条件的 latent world model”。

### 情况 B：Future-BEV 不优于几何 Warp

不强行保留 future-BEV 作为核心创新。可改为当前 BEV + 学习动力学 Warp，并将重点转向真实动力学和规划不确定性。

### 情况 C：联合模型不优于独立串联

不能宣称联合建模贡献。应检查跨模态交互、数据覆盖和参数公平性；若复核后仍无优势，论文定位改为球形机器人数据驱动导航系统。

### 情况 D：离线指标提升但导航不提升

检查代价函数、规划器是否利用了改进信息，以及模型是否存在规划可利用漏洞。导航闭环仍无收益时，不能将预测提升表述为实际导航贡献。

### 情况 E：当前数据不足

保留现有数据用于管线测试，不从小数据结果得出论文结论；优先补采多样动作和空间监督，而不是通过重复采样扩大表面样本量。

---

## 19. 最近四个可执行里程碑

### M1：数据可用性

- 完成 manifest；
- 修复或排除异常 episode；
- 明确视觉帧之间的动作窗口；
- 给出现有数据可用于哪些健康检查的结论。

### M2：空间教师

- 新采或重放一批带深度数据；
- 生成正确的三分类 BEV；
- 完成 100 帧人工抽检和几何单元测试。

### M3：三项健康检查

- 历史动力学优于简单基线；
- 双目 LeWM 无坍塌；
- 当前 BEV 头能够稳定解码障碍。

### M4：最小联合模型

- 统一输入视觉、状态和动作；
- 一步输出未来状态、latent 和 BEV；
- 完成联合模型与独立串联模型的初步比较。

完成 M4 后再决定是否投入完整多步训练和实物规模数据采集。

---

## 20. 一句话执行原则

> 最终做一个联合世界模型，但按“数据正确—单项可学—联合单步—联合多步—闭环导航—实物验证”的顺序推进；任何阶段未通过门槛，都先解决当前问题，而不是继续增加模型复杂度。
