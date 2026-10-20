# 球形机器人动力学适配轨迹跟踪器 V1 设计

## 目标

在不改变 Hybrid A*、障碍物地图、动作协议、PD、正式成功判据和 35 s 预算的前提下，新增独立的 `oracle_trajectory_v1` 跟踪策略。它把现有几何路径转换成连续曲率与速度剖面，以曲率前馈为主、误差反馈为辅，减少 Pure Pursuit 在约 1 s 转向响应迟滞下的反复大幅修正。

旧 `oracle`/`OraclePathPolicy` 必须保持原样，作为冻结基线。V1 不是世界模型策略，也不使用全局地图以外的新特权信息；它只消费 Oracle 已有的 `kinodynamic_reference`。

## 已确认的依据

- 固定零指令、固定前进和固定 ±10° 副轴角时，振荡会衰减；没有证据支持先扫 PD。
- 副轴从应用目标到首次达到目标 90% 约需 0.86–0.98 s；10° 目标还包含约 0.30 s 的目标限速过程。
- Pure Pursuit 前视 0.5 m 会产生逐渐放大的横向/roll 振荡；1.0 m 明显稳定，但正式 `hard_0` 仍只有 2/7。
- 去掉现有横摆角速度反馈没有一致收益，不能作为主修复。
- 跟踪验收的曲线路径 waypoint 中 `auxiliary_angle_rad=0`，因此 V1 的权威曲率来源必须是 `(x,y,yaw)` 几何；Hybrid A* waypoint 动作用于一致性校验，不能成为唯一来源。

## 前置条件：确定性复位

当前 runtime 在设置 episode pose/零速度后调用相机预热，而相机预热会推进数量可变的物理步。随后才建立相对时钟并捕获 state 0，所以不同进程可能具有相同相对时钟、不同实际关节速度。

修复顺序固定为：

1. 完成 `_prime_stereo_cameras`；
2. 再次写入 episode pose、零 root 线/角速度、零关节位置/速度；
3. 重置 `ActionSmoother` 与 `previous_action`；
4. 执行一个固定的 `world.step(render=False)` 使状态生效；
5. 建立 clock epoch；
6. 捕获 state 0。

不得清空相机缓存或再次执行可变长度预热。测试必须证明预热次数不同不会改变 state 0 的物理字段；raw epoch 可以不同，但相对 episode 时钟仍从零开始。

## 空间参考剖面

新增小型、纯 CPU 的 `SpatialReferenceProfile`，从同一 drive-direction segment 内的 waypoint 构造：

- `xy[N,2]`；
- `yaw_unwrapped[N]`；
- 单调累计弧长 `s_m[N]`；
- 几何曲率 `curvature_1_m[N]`；
- 原始 `drive_direction` 与 `main_command_rad_s`。

几何曲率以 `d(yaw_unwrapped)/d(s)` 计算：内部点使用中心差分，端点使用单边差分；重复终点继承相邻有限值。每个方向段独立展开 yaw，绝不能跨换向点平滑。随后在同一方向段内按 0.40 m 弧长窗口做局部加权平均，减少 Hybrid A* 离散航向造成的曲率跳变。

无效条件必须失败：少于两个有效点、非有限坐标、非单调有效弧长、未知方向。直线曲率应为 0；半径 3 m 的左/右 90° 圆弧应分别接近 ±1/3 m⁻¹；S 弯应连续换号；换向段不能相互污染。

## `TrajectoryOraclePolicy`

新增策略名 `oracle_trajectory_v1`，复用旧策略的：路径投影、方向切换停稳门、终点判断、DriveModel、动作协议和安全失败状态。旧类不重构、不继承新行为。

每个 10 Hz action block：

1. 将当前位置投影到当前方向段，仅允许路径索引单调前进；
2. 选择至少 1.0 m 的空间前视点，速度自适应前视仍作为下限补充；
3. 在前视区间读取平滑参考曲率 `kappa_ff`；
4. 根据参考切向计算带符号横向误差 `e_lat_m`，根据参考 yaw 计算包角后的航向误差 `e_heading_rad`；倒车时以运动方向切向计算横向符号；
5. 形成有界曲率修正：

   `kappa_fb = clip(-0.20 * e_lat_m + 0.15 * sin(e_heading_rad), -0.20, 0.20)`，单位均归一为 m⁻¹；

6. `kappa_cmd = kappa_ff + kappa_fb`，再由 `DriveModel.auxiliary_for_curvature(main_command, kappa_cmd)` 转换为副轴角；
7. 现有命名的 yaw-rate feedback（默认 0.15 s）只允许在副轴域提供 ±5° 内的小修正；整个反馈修正不得突破动作协议 ±30°，最终仍经过现有 30°/s limiter；
8. 保存 `kappa_ff/kappa_fb/kappa_cmd/e_lat/e_heading/main_cap/aux_ff/aux_feedback` 诊断值。

上述增益是 V1 的保守起始值，不视为已标定生产值。全部必须显式写入配置与产物；本轮不做网格扫参。

`latency_preview_s` 接口可存在但默认且首轮固定为 `0.0`。在获得按方向、主轴速度和副轴角分组的响应表之前，不允许把 0.9 s 写成默认预测延迟。

## 曲率前瞻速度剖面

速度选择不再只读取当前 waypoint。策略检查未来 0.80 m 内 `max_abs_curvature`，取参考主轴幅值与以下上限的最小值：

- `<0.20 m⁻¹`：上限 0.50 rad/s；
- `[0.20,0.45) m⁻¹`：上限 0.35 rad/s；
- `>=0.45 m⁻¹`：上限 0.25 rad/s；
- `|e_lat|>=0.15 m` 或 `|e_heading|>=20°`：额外上限 0.25 rad/s。

前进/倒车只改变符号，不改变幅值规则。所有变化继续经过现有主轴 0.5 rad/s² 加速度限制，因此“弯前减速”必须来自 0.80 m 前瞻，而不是进入弯道后才降速。

终点与换向第一版保留冻结的 stopping-distance 和五周期停稳门，不同时重写制动控制；到达后的 hold 单独评估。这样本轮只回答路径跟踪与转弯速度问题。

## 集成与产物

- tracking acceptance 增加 `oracle_trajectory_v1` policy variant，默认仍是旧 baseline；CLI 和 child command 必须记录全部 V1 参数。
- formal benchmark 只有通过开放平面门后才增加同名 policy choice；仍使用普通 kinodynamic plan，不走 `oracle_v2` dynamic plan。
- runner 不新增参考类型。策略 diagnostics 按 block 写入 metrics/artifact；产物验证器同步更新。
- 所有失败在关闭 runtime 前保存 partial；不覆盖已有 run root。

## 分阶段验收

### CPU/契约门

- 曲率符号、方向分段、重复终点、速度上限、反馈单位和限幅均有手算测试；
- 旧 `oracle` 默认动作逐元素保持不变；
- deterministic reset 的 fake-world 测试覆盖不同相机预热步数；
- CLI/config/child command/metrics 中的参数一致。

### 开放平面门

同一冻结源码下，独立进程运行旧 `oracle` 与 `oracle_trajectory_v1`：直线 0.5、左右 R3 圆弧、S 弯、倒车、混合换向。每对必须 state 0 物理字段一致；否则该对作废。

候选进入 formal 的条件：

- 直线和倒车的最大横向误差不得比基线恶化超过 0.02 m；
- 左右 R3 与 S 弯的最大横向误差至少两项降低 20%，并且不能新增 arrival/hold 失败；
- 无碰撞、filtered contact、runtime error 或时钟错误；
- 主轴减速与副轴变化曲线证明动作连续，没有靠提前停止获得低误差。

### 正式门

仅候选通过开放平面门后，在冻结的 `hard_0` 七任务上与 lookahead=1.0 的 2/7 基线比较。地图、规划路径、0.1 m 裕度、成功/碰撞/超时定义和 35 s 预算不变。目标是成功数不下降、碰撞少于 4/7，且逐任务轨迹偏离不恶化；否则保留旧 baseline，并根据失败轨迹再迭代。

## 明确延期

- MPC、LQR、学习型跟踪器和世界模型控制；
- 非零固定延迟预测；
- timed trajectory/reference schema；
- PD、动作 limiter、地图和规划器调参；
- 用开放平面结果宣称正式导航成功率提升。
