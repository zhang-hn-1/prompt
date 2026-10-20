# 当前相机朝向反馈观察：实际运行与拒绝/准入分层

本次判断 **REJECT_CAMERA_HEADING_OBSERVATION_PROBE**。这是一个真实闭环开发诊断，整个研究未完成，没有触发正式E0/E1/E2，也没有新增导航成功率。

## 问题和唯一改动

上一版速度方向反馈在事件后0.2秒已经改变10.2048°，相机光轴水平朝向只改变0.7213°，导致转向提前结束。它符合旧course规格，但代理量没有完成观察目标。本次只将观察反馈改为当前左相机光轴在世界XY中的朝向，光轴=-本次USD相机world transform的Z列。该transform来自既有capture事务，未增加物理步/相机读取/policy调用。

这是**真位姿特权诊断**，不是纯RGB估计位姿。图像模型仍仅读取当前双目RGB。相机位置、照明、Fast权重、PID、事件两个不同源不足100、原始停稳五次/反弹规则、+.5主轴/−30°副轴请求、目标+10°、4秒转向硬截止、6秒观察硬截止、完整35秒预算全部冻结。水平投影退化明确NOT_EXERCISED，不能发明方向。窗口结束后不能锁存成功，不等待、不重置、不触发第二次观察。

预先冻结计划：`docs/superpowers/plans/2026-10-07-camera-heading-observation-correction.md`。只一次实际完整序列，无科学重试/幅度时长扫描/模型或门槛救分。

## 实际执行和终态

实际驱动开始2026-10-07T05:47:56.585075+00:00，结束2026-10-07T05:53:39.877100+00:00，壁钟343.292s；终态原始指标如下。任务是相同2.6m局部开发参考、yaw−20初态，不是原完整正式场景，继承Oracle元数据不用于SPL。

```json
{
  "episode_id": "observation_handoff_e1_observe",
  "scene_id": "e1_detour",
  "difficulty": "E1",
  "policy": "experimental_rgb_knownpose_flat_ground_prior_localgoal",
  "termination_reason": "collision",
  "success": 0.0,
  "collision": 1.0,
  "timeout": 0.0,
  "runtime_error": 0.0,
  "action_blocks": 312,
  "elapsed_s": 31.2,
  "final_distance_m": 2.0129292465092443,
  "path_length_m": 2.459408594182793,
  "reference_length_m": 8.836753236814713,
  "spl": 0.0,
  "goal_progress_m": 0.5870711886046682,
  "minimum_clearance_m": 0.012310562561766047,
  "time_to_goal_s": null,
  "runtime_error_message": null,
  "diagnostic_stop_hold_passed": null,
  "diagnostic_goal_entry_action_block": null,
  "diagnostic_settling_blocks": 0,
  "diagnostic_settling_runtime_error": null
}
```

事件发生19.999999552965164s；实际相机目标达到=True，相机水平朝向最大变化19.5018310546875°，3D光轴最大变化19.433081877220403°，真实根部最大位移0.6506659969146749m。首次目标记录：{"frame": 226, "time_s": 22.599999494850636, "turning": false, "camera_heading_goal_reached": true, "feedback_heading_change_deg": 10.245404467131424, "camera_optical_heading_change_deg": 10.245402336120605, "camera_optical_axis_3D_change_deg": 10.19118875754028, "camera_elevation_deg": 5.968291439099659, "root_displacement_m": 0.4224130928111443, "camera_displacement_m": 0.38606423139572144}。不能用后来恢复B0的运动冒充观察命令的标签。无区间接触=True；没有接触不等于导航完成，观察占用预算可能延后失败。

## 全部观察窗口源测量

每个非空包都要求左右两个实际native像素端点的世界误差<=.10m且精度>=.95，数据不足继续留在分母；至少两个不同更新100点、新区域20点/2格，无区间接触。离线renderer标签只在源/请求封存后评分，不进入决策。全部窗口合计{'n': 389, 'correct': 337, 'insufficient': 0, 'finite_wrong': 52}。

|帧|秒|源点|两端正确|不足|有限错误|全体正确率|
|---|---|---|---|---|---|---|
|205|20.5|337|337|0|0|1.0|
|210|21.0|0|0|0|0|N/A|
|215|21.5|0|0|0|0|N/A|
|220|22.0|1|0|0|1|0.0|
|225|22.5|0|0|0|0|N/A|
|230|23.0|0|0|0|0|N/A|
|235|23.5|0|0|0|0|N/A|
|240|24.0|0|0|0|0|N/A|
|245|24.5|1|0|0|1|0.0|
|250|25.0|50|0|0|50|0.0|
|255|25.5|0|0|0|0|N/A|
|260|26.0|0|0|0|0|N/A|

每非空包95%=False；两个100点=False；新20点/2格=True。单个好包不能代替全部窗口验收，观察执行通过也不意味着深度、地图、控制或导航通过。

## 接口、执行与实际证据

作者实际运行六项意义回归和完整522项测试（24.24s）。最初六个RED为未实现；首次GREEN尝试有1项测试将旧source205/20.5误写为新207/20.5，时钟保护拒绝，保留失败日志后修正测试身份。无放松生产保护。首次独立审查C0/I0/M1，审查者个人六测试与1295源SHA、实际diff、窗口外不锁存；完整suite/物理运行由作者个人完成。唯一Minor旧schema名称由单独effective scope覆盖，旧协议字节与旧scope均保留，源码行为不变。

collection首次因未配置USD库缺pxr失败，保留日志，仅补环境后同一只读核验通过，没有重跑物理或改变科学输出。

实际接口验证312请求，所有当前轴/名义请求/最终请求/事件字段逐条重建；前缀截断输出相同，未来源/轴时钟在读取payload前拒绝。原始handoff、smoother/PID/supervisor在**实测反馈**下重建一致，不是无未来真值递归物理预测。源包/双目CAD/当前地图与只读shadow逐字段重建，源与输出SHA/曝光时钟核验，视频逐帧实际解码。验证脚本和JSON在独立camera_heading命名空间。

与上一版分叉前的实际前缀：

```json
{
  "utc": "2026-10-07T05:54:48.488194+00:00",
  "through_event_frame": 200,
  "physical_state_prefix_exact": false,
  "namespaces": {
    "online_inputs": {
      "equal_fields": 201,
      "unequal_fields": 402
    },
    "known_poses": {
      "equal_fields": 573,
      "unequal_fields": 30
    },
    "history": {
      "equal_fields": 553,
      "unequal_fields": 50
    },
    "policy_inputs": {
      "equal_fields": 1291,
      "unequal_fields": 518
    }
  },
  "first_mismatches": {
    "online_inputs": [
      {
        "frame": 0,
        "field": "left_rgb"
      },
      {
        "frame": 0,
        "field": "right_rgb"
      },
      {
        "frame": 1,
        "field": "left_rgb"
      },
      {
        "frame": 1,
        "field": "right_rgb"
      },
      {
        "frame": 2,
        "field": "left_rgb"
      }
    ],
    "known_poses": [
      {
        "frame": 186,
        "field": "left_camera_usd_world"
      },
      {
        "frame": 186,
        "field": "right_camera_usd_world"
      },
      {
        "frame": 187,
        "field": "left_camera_usd_world"
      },
      {
        "frame": 187,
        "field": "right_camera_usd_world"
      },
      {
        "frame": 188,
        "field": "left_camera_usd_world"
      }
    ],
    "history": [
      {
        "frame": 181,
        "field": "action_history"
      },
      {
        "frame": 182,
        "field": "action_history"
      },
      {
        "frame": 183,
        "field": "action_history"
      },
      {
        "frame": 184,
        "field": "action_history"
      },
      {
        "frame": 185,
        "field": "action_history"
      }
    ],
    "policy_inputs": [
      {
        "frame": 0,
        "field": "left_rgb"
      },
      {
        "frame": 0,
        "field": "right_rgb"
      },
      {
        "frame": 1,
        "field": "left_rgb"
      },
      {
        "frame": 1,
        "field": "right_rgb"
      },
      {
        "frame": 2,
        "field": "left_rgb"
      }
    ]
  },
  "strict_prefix_certified": false,
  "posthoc_only": true,
  "no_future_control_information": true,
  "no_navigation_admission": true
}
```

如果strict_prefix_certified为false，只能作同任务历史比较，不能声称严格唯一因果A/B；当前实验内部单一改动的代码差异和封存身份仍明确。

## 分层结论与后续边界

反馈语义修正、观察是否实际转向、源几何、可执行导航是不同层。当前判断与上述所有冻结门槛分别保留；旧course、neutral、Hero、preLR及M2拒绝均不追改。不能通过延后碰撞、停住、路径索引跳转、未知填自由或单点稳态通过宣称视觉导航成功。

若本候选拒绝，不继续扫观察角度、时长、增益或替换网络。下一项工作必须依据本次实际可见空间/源输出/执行证据选择，并继续保留未知、当前观测、历史观测和预测的来源差异；估计位姿、学习记忆闭环以及原35秒E0/E1/E2尚未准入。本报告不是整个任务完成通知。

实际图像与路径：

![实际路径、源测量及bank](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/camera_heading_observation_causal_audit_v1/actual_visibility_and_bank_v1.png)

![实际native双目](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/camera_heading_observation_causal_audit_v1/actual_native_stereo_snapshots_v1.png)

[双目视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/camera_heading_observation_run_v1/stereo_view.mp4) · [第三人称视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/camera_heading_observation_run_v1/third_person.mp4)


## 2026-10-07T06:07:19.441023+00:00 全曝光共同视野丢失分解（GT-only后置）

不是仅仅网络停止输出，也不能仅归因bank。用全部12实际曝光的真实左点投影到**当时真实右相机**，按右FOV、固定baseCAD、动态新增CAD与真实同表面分层，得到下表。X/Y出界可重叠，表中的“右出界”取并集。未改变任何gate或源码动作，0新采集/推理/物理。

|帧|左眼实际可见候选|右FOV外|固定base挡|动态新增挡|真实共同表面|左axialdepth中位m|
|---|---|---|---|---|---|---|
|205|25643|19137|4406|427|1671|1.2855|
|210|25985|20261|4273|401|1050|1.2219|
|215|34531|28703|5701|113|14|1.0990|
|220|38716|33370|5275|70|1|0.9730|
|225|40110|36552|3558|0|0|0.9028|
|230|40247|39372|875|0|0|0.8449|
|235|39970|39947|23|0|0|0.7836|
|240|40048|39921|127|0|0|0.7624|
|245|39675|39675|0|0|0|0.7795|
|250|39002|39002|0|0|0|0.7883|
|255|39641|39641|0|0|0|0.7819|
|260|39757|39757|0|0|0|0.7709|

205→260左实际候选仍25643→39757，但右共同表面1671→0。225尚有3558点在右FOV内，全部落在**固定base**遮挡区域；245..260全部因native垂直伙伴坐标出界。动态新增mask没有独占解释这个损失。伴随转向的真实平移使左障碍axialdepth中位从1.2855m降至约.771m；这是实际接近和视野变化共同发生的证据，不是孤立bank因果A/B。

当前native垂直基线.60m、f584.8889、480像素高，必要公式 `right_y=left_y-fB/Z`；实际两camera world pose复算与该公式最大差0.000098118px。即使完全不考虑遮挡，理想全画幅最乐观Z下限0.732637m，中心行约1.465275m；不是“.5..3m里面每个点都可以双目看见”，更不是准确/可靠或自由体积保证。250的50个有限错误，真实右伙伴全部已出界；这解释该包为什么不能信任，不是重新筛点救分。

运行中曾按旧版205帧参考作快速只读朝向测量，口头给出的22.5秒/10.31°只是相对于该参考；封存请求重建显示**本次事件是200帧20.0秒，首次真正目标是226帧22.6秒10.2454°**。最终结果以实际事件记录为准。源RGB从0帧不同、请求历史181开始不同、已测相机pose186开始不同，strict前缀不成立，不能把碰撞31.2对旧版35超时写成唯一因素因果提升/下降。

collection只读核验输出还有继承的 `neutral_bank_probe:true` 元数据笔误；独立camera_heading_observation_collection_semantics_scope_v1.json绑定原JSON哈希并明确effective camera-heading=true/neutral=false。312请求、63源、1386packet/mask、1872地图、3744shadow、3884SHA、曝光0、1561physics、6接触及两313帧视频等科学字段未改变。作者已实际打开两张输出图。旧report字节SHAde20902ad889e0c3e24ea2c0d5fd5404077439b9c5001fa11ed978691cad3dbf、bytes9302；旧结果seal保持，不冒称覆盖新附录。
