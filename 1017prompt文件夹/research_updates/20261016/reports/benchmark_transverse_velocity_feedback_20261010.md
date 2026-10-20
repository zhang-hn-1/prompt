# 正常前进横向速度反馈：实际E1对照

仅在当前有效正常前进且机体/参考方向处于同半球时，将参考法向的实测XY速度乘固定1 rad/(m/s)加入父控制器最终副轴请求。保持规划、完整参考、主轴、roll-PID、相机、光照、换向规则和35秒规则。没有重新训练模型、调增益或扫阈值。

| 实际指标 | 同次B0 | 唯一候选 |
|---|---:|---:|
| 净目标轴进度/m | 0.975779 | 0.970787 |
| 距目标/m | 5.626473 | 5.630939 |
| 实际总行程/m | 4.769825 | 4.760905 |
| 横向RMS/m | 0.335218 | 0.342927 |
| 横向峰值/m | 0.721184 | 0.741894 |
| 真实碰撞 | 0 | 0 |
| 35秒内到达 | 否 | 否 |

候选80次真正改变请求，首次分叉为frame61，之前请求、非raw物理、完整执行状态与初始重置一致。两臂每条均350请求、351相机采样、1751物理采样、2101接触检查、2211封存文件；双视频各351帧10Hz完整解码。两次实际进程正常退出0，没有超时杀进程或重试。九项CPU测试通过，2179源码与110输入SHA一致，独立启动前和闭合审查均C0I0M0。

arrival与net_progress门槛失败，候选拒绝。约5mm的净进度损失不能解释为可靠改善，也不支持再扫反馈增益。已知位姿和当前世界速度明确属于诊断特权；没有新增估计位姿E0/E1/E2结果、模型或严格纯视觉准入。源像素前缀尚未证明，不作完整因果改善声明。

真实分段没有改变：观察6s，前进6s，第一次制动2.5s，倒车16.4s，第二次制动1.2s，最后前进2.9s。B0首前进净增加1.554m，倒车净减少1.552m；候选对应1.551m与−1.549m。因此单独增加横向阻尼没有解决长倒车消耗时间和撤销净进度的问题。不同阶段及不同参考下的横向速度统计只能作描述，不能冒充同状态物理标签。

实际执行链进一步解释了早期请求差异为何没有立即运动差异：反馈desired/raw PID首次不同为tick305，protocol PID首次不同tick333，smoothed/applied首次不同tick354，对应物理sample355、7.1s。请求从6.1s分叉，而305–353共49个反馈tick的applied仍完全相同。共同PID饱和与平滑斜率限制屏蔽了部分早期差异；这不证明实现bug或导航失败的唯一原因。后续真实applied、关节、XY确实不同，候选不是全程行为等价。

个人核验：实际启动/监督两臂、评分、视频逐帧解码、执行链索引和分段审计，并查看轨迹图片。未声称人工观看完整视频。独立执行链补审C0I0M0。

证据：experiments/stereo_memory_navigation_20261004_v1/transverse_velocity_feedback_e1_assessment_v1.json、transverse_velocity_phase_audit_v1.json、transverse_velocity_execution_effect_v1.json、transverse_velocity_first_effect_lineage_v1.json、transverse_velocity_feedback_e1_closed_review_v1.json、transverse_velocity_execution_effect_review_v1.json。视频在 transverse_velocity_feedback_e1_{b0,candidate}_run_v1/{stereo_view,third_person}.mp4，轨迹为 transverse_velocity_feedback_e1_trajectory_v1.png。

整体研究未完成，旧模型、学习记忆覆盖和历史导航拒绝保持。下一项直接利用本次新保存的每次调用完整参考身份，独立分段核验实际几何进度与参考重建；不调用旧solver、不重跑矩阵、不扫增益，不把path_index变化当真实进度。
