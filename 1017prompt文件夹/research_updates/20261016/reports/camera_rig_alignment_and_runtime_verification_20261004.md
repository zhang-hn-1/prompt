# 相机安装位置统一与实测核验（2026-10-04）

已将主仓库及 e1-detour-benchmark 工作树的运行相机安装位置统一为用户指定的最新值。坐标属于球心处的 base_link 局部坐标系，相机随 base_link 运动。

| 参数 | 原运行值 | 当前运行值 |
|---|---:|---:|
| 前方 X | 0.10 m | 0.20 m |
| 左/右 Y | ±0.30 m | ±0.30 m |
| 上方 Z | 0.10 m | 0.05 m |
| USD rotateYZX | (0, -90, 0)° | 保持原值 |

两个代码目录各自增加同内容的 scripts/camera_rig.py，安装版本为 ball_stereo_front020_up005_v2。双目间距仍为 0.60 m；焦距、像面尺寸、分辨率及控制链未修改。

覆盖入口：collect_data_dubins、旧 collect_data、正式 benchmark 的 Isaac runtime、世界模型导航 evaluate_navigation、交互 sim_env。交互入口保留显式坐标覆盖选项。正式 benchmark 的运行记录新增 camera_rig_installation，便于区分新旧安装。

## 测试与实际运行

位置回归测试在修正前复现五个入口的 0.10/0.10 m 错误；修正后五个入口均通过。测试直接执行各入口的 USD 相机创建函数，检查左右位置、旋转、内参及交互参数覆盖。

| 测试范围 | 结果 |
|---|---|
| 工作树：相机及采集/导航/深度/契约相关回归 | 265 passed，64 skipped |
| 主仓库：相机及采集/导航/契约相关回归 | 175 passed |
| 工作树：相机及 benchmark runner 回归 | 49 passed |
| 主仓库：相机及 benchmark runner 回归 | 13 passed |

以上为四次测试命令的结果，其中相机测试重复执行；64 项跳过不视为通过。实际 GPU 验证独立启动 E1、E2 runtime，完成 reset 后同步渲染并读取 USD 局部变换及 optical-to-base_link 外参。没有运行导航评测，也没有产生新成功率。

| 场景 | 眼睛 | 实测局部位置 (m) |
|---|---|---|
| golden_e1_0001 | left | 0.199999809, 0.299999982, 0.050000012 |
| golden_e1_0001 | right | 0.199999809, -0.300000012, 0.049999982 |
| golden_e2_0001 | left | 0.200000286, 0.299999982, 0.050000012 |
| golden_e2_0001 | right | 0.200000286, -0.299999982, 0.050000012 |

实测外参符合 (0.20, ±0.30, 0.05) m，局部平移核验容差为 1 微米；光学坐标旋转与原实现一致。两次拍摄均检查相机时间与当前物理时间相同，额外渲染没有改变物理状态。一次检查因 0.19 微米浮点误差超过最初的 0.01 微米容差而停止；放宽到预先明确的 1 微米数值容差后完成核验，保留该失败诊断。

## 新安装对应的派生数据

已重新投影机器人 CAD 生成 base_link_masks.npz，左右掩膜均为 480×640 布尔数组，覆盖 base_link 固定部件并扩张 3 像素。新位置的掩膜覆盖约 46.24% 像素，只覆盖固定部件，不能用于替代可动部件或障碍感知。

掩膜和身份记录已装入两个代码目录的 configs/camera_rigs/ball_stereo_front020_up005_v2/。主仓库和工作树所用机器人 USD 四个层文件的 SHA256 一致。后续新视觉实验应读取该目录的新掩膜及真实相机外参。

历史实验的图像、数据、报告、权重及冻结几何脚本保留原安装身份；旧结果不能重新标成新安装验证。历史实验脚本中的硬编码投影和旧掩膜属于旧数据复现组件，不应直接接到新的 runtime。

本次验证范围是安装参数与实际渲染一致性；视野遮挡和视觉导航性能仍需在新配置下后续验证。

## 证据

- [当前 E1/E2 双目原始照片拼图](/home/an/Documents/ChatGPT/球型机器人导航/experiments/camera_rig_alignment_20261004_v1/current_stereo_views.png)
- [最终来源与实测核验](/home/an/Documents/ChatGPT/球型机器人导航/experiments/camera_rig_alignment_20261004_v1/final_verification.json)
- [局部源代码差异](/home/an/Documents/ChatGPT/球型机器人导航/experiments/camera_rig_alignment_20261004_v1/camera_source_patch.diff)
- [新掩膜身份](../experiments/camera_rig_alignment_20261004_v1/mask_identity.json)
