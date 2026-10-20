# 整序列动作/状态覆盖采集：停止结论

冻结30组上限，实际只运行2组；首覆盖失败即关闭，不补跑剩余28组，不改程序/阈值救分。首组E0：70/70完整；第二组E1：57/70=81.43%完整（不是57/58）。第58窗口在28.78秒接触，原动作块28.80秒末停止，多执行1个物理tick，如实保留；25预提交仅15实际消费，标删失。

两child正常exit0，无timeout/中断；它是数据资格拒绝，不是导航基线性能结论或学习模型性能失败。2282源/输入SHA、两组86/74封存文件、native重建、RGB时钟/字节、动作/监督与四视频均独立复核。两组都只development，calibration/evaluation未启动，fit/model/teacher为0。

原数据和commands见bound_schedule_v1.json、dataset_admission_v1.json、各group seal/status与runs目录；dataset_assessment_v1.json和independent_actual_closure_v1.json为闭合结论。监督器已有固定collection_status，禁止重复启动。旧zero pilot1/1与其他旧预算保持关闭。

位姿为原生IMU/滚动模型+初始GT规范/地面CAD先验；低层rollPID沿用理想IMU/仿真角速度特权。RGB只作为固定相机字节来源，metadata不是pixel freshness证明，不能称纯视觉或泛化保证。未来GT仅监督。

研究目标仍未完成：可靠导航和学习导航增量未成立。下一转任务六权威可复现交付与运行/资源范围核对，不从负结果派生控制诊断或扩采集。
