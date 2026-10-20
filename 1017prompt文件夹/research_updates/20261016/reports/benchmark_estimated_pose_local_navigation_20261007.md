# 双目＋理想惯性/编码器估计位姿局部导航

唯一实际局部2m episode，原35s最大规则：22.8s真实到达、终距.199806685m、接触0、runtime错误0；228请求/229capture/1141物理/46双目推理。相机安装/照明/网络/rollPID/未知速度cap.3/局部planner均未改。目标在原E1障碍前，不能称完整E1、正式成功率或已验证绕障。

与旧knownpose current22.6s/.193028772比较，本次没有性能改善声明。两次RGB重置不是同流；传感器接口和端点不确定性为本次整体可执行位姿消费，不能从两个episode断言单项因果收益。

导航真实运行中使用估计相机世界矩阵、地图点、根位置、航向、速度、角速度、重力/四元数历史和从已知世界goal重算的点目标。原导航位置/yaw/动态quaternion、17D中0:9真值运动列及runtime真实点目标不再读取；9:17关节/编码器/请求和实际action/RGB允许。一次t0真实初始全局标定、理想模拟bodygyro/主轴编码器、底层rollPID理想测得roll是明确特权。gyro模拟转换器内部当前真姿态属于理想传感器边界，非纯RGB/真实有噪IMU验证。

作者审计1264先验源SHA与1976关闭产物SHA、2061估计相机/状态字段、460端点字段、1368导航观察字段重建；全部1141在线sensor/预测与冻结因果replay精确。PolicyObservation正式contract会把17D与quaternion转float32，审计按相同cast逐项精确，未用宽容误差提高成绩。所有地图源注册只读当时曝光的估计姿态。

三个独立gate均PASS：真实任务到达/无碰撞；沿冻结直线目标的真实运动tracking RMS 0.048796549m/peak 0.128785110m，小于.12/.30；完整t>=1s1090行原3D RMS 0.046172205m/peak 0.046574049m、姿态p90 0.686907882°，小于.05/.10/2。46mm高度误差未夹紧。真实goal方向末态进度 1.847235125m，终点横向偏差有贡献，不能把真实2m goal entry写成投影恰走2m。

源端点sigma加.10+range*sin(2°)只是来自准入容差的启发式敏感性，不是统计置信/安全保证；地图保持not-certified、地面prior不是观测。旧raw深度/高速Oracle/M2/完整35s导航失败均保留。policy类名与metrics.policy继承字符串仍写knownpose，不能据此认定运行权限；本次源码派生身份、actual输入重建和identity估计标记为范围证据。

2个观察隔离与4个桥接测试先RED后GREEN，正式PolicyObservation fixture通过，完整405测试20.69s；两视频各229帧完整解码。全部原始数据、图和代码保存。没有新增独立终审：agent-thread上限已有失败，本阶段author-only，不借其他分支联合review认证。

完整目标/真实绕障、记忆增量、学习增量和原35秒E0/E1/E2尚未通过。下一项必须利用末段current失去障碍而history保留的现有实测证据，分开定位、记忆和运动预算，不能把本局部前进到达冒充最终研究交付。


## 单次独立终审追加（2026-10-07）
ONE fresh reviewer C0/I0/M1：17项限定测试.64秒、13源/11产物身份、实际PolicyObservation四帧poison重建及独立数值复算；完整405suite/全身份/全字段/视频仍为作者证据，不冒称审查者重跑。横向评分窗口明确为 t>=6−1e−6 的841行；原定位窗口 t>=1 的1090行。只接受理想传感器局部估计位姿接口可行性，不认证完整E1/纯RGB/原生IMU/独立memory分支。审查文件 estimated_pipeline_review_final.md 与 personal_checks.json。追加前报告SHA256：9e197e109499e42e41d8ec2c1851cc0782f2d6a43a1ef68427ac690b963d97df。
