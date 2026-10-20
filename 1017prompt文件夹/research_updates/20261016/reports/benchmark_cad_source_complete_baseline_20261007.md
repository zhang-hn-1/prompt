# CAD过滤双目源的原完整目标：35秒实际失败与同流记忆接口

ONE真实原6.6米目标/35秒测试结束：超时，终距4.749973411米，零接触/无运行错。真实沿起点→目标直线进度1.878816766米，峰2.190476459米，路径3.573071290米。它仍是RGB+真位姿/groundprior/受控未知探索诊断，**不是纯RGB估计位姿或正式E0/E1/E2成功率**。任务整体未完成。

## 目的、版本与实际运行

原.3/current baseline及35秒保持；唯一在线来源变化为已经2米局部验证的current-joint CAD过滤。与旧完整oldmask/current相同class/旧PID/main acceleration/副轴速度/参考消费者/BankMap/未知限速，camera/light/model不变。它不是被拒cap1.0的完整goal准入：cap1.0终距2.35mm gate拒绝、full未触发保持；本次基线仍.3，不救该分。

spec/plan `2026-10-07-cad-source-complete-baseline*`，runner `run_cad_complete_baseline.py` 与旧run_rgb_complete_goal.py逆派生精确；1306项 source preseal。5source/current-input fixture3.26秒、global符号检查clean。UTC 2026-10-06 21:15:31.542139→21:25:11.380883，exit0，唯一35秒物理序列，350act/351双RGB/1751physics/71推理，无额外尾帧/重跑。wall约580秒，非实时；model wall p50/p95=1.797/2.077秒，historyshadow p95=1.665秒。

## 同实时masked源记忆shadow

primary current；history10s shadow独立policy，每frame消费同masked source与真实current/past反馈，350调用/228raw请求不同、首6秒。shadow从未执行，不把actual current后续轨迹作为不同history计划的标签。每次断言原primary tracking边界/request/clock/完整物理pose/velocity/joint readback不变；没有完整独立物理shadow-off run。source当前或过去，最高age .399999991秒；futurepayload trap通过，曝光delta0。

与旧独立实际oldmask/current比较，首请求分叉7秒（70帧），前351physics×17非raw fields和71execution快照精确，共350块261请求不同；RGB重置本身不是逐字相同，不能只凭source唯一代码改动把整条性能差纯归因于mask。旧baseline35秒/终距4.489753/进度2.112201本来失败；新source-filter不能宣称改善，终态更差。没有改写旧结果或继续扫mask/速度。

## 完整失败分段：真实进度而非索引

| 实际请求段 | 时段s | 时间s | 进入→退出真实沿目标进度m |
|---|---|---:|---:|
|观察/落地|0–6|6|0→.033248|
|非零跟踪|6–26|20|.033248→2.116076|
|零请求制动|26–28.5|2.5|2.116076→2.173489|
|非零，main负向请求|28.5–31.3|2.8|2.173489→1.957993|
|零请求制动|31.3–33.7|2.4|1.957993→1.822746|
|再次非零前进|33.7–35|1.3|1.822746→1.878817|

总60观察/241非零/49trackingzero，0 no_route。是实际请求与真实XY分段，不用path_index/跳参考当进展。26秒停走之后存在真实倒退，最后复行，不能称永久停止/碰撞。换向资格规则未改，单独这张表不是完整stop-counter复现。记录中guard_measured_this_call=false的分支，不把陈旧参考guard/曲率诊断当当时实请求；最终local.request与physical applied为动作权威。

前20秒非零跟踪只取得约2.083米目标方向进度；全序列XY速度p95 .166863m/s，而6.6/29=.227586是名义直线预算。它支持“当前执行速度与后段停走都限制35秒进度”的有限诊断，不是全域速度上界或不可能性证明，也不预先认为合理制动就是bug。旧BankMap低速域匹配问题仍未解除。本次没有只靠扩超时/停在障碍前得分。

## 数据、来源与真正核验

71source旧base144961点→CAD140008，删除4953：3374准确≤.1米、813有限错误、766renderer标签无效。不是删点全错误/源准确性通过，旧strict depth gate保留。CAD只用当前public3关节、静态CAD/rig；GT深度在source包确定后用于独立offline误差，不进policy。task ground/safe-start是prior标签，未观察free保持未知；knownpose explicitly privileged。

author posthoc实际4003SHA=1306preseal+2697postclose，71来源/mask1562fields，350currentmaps2100数组、350historyshadow4200诊断，futuretrap，完整两视频各351帧10fps逐帧解码。posterior地图提取用了独立grid-only优化，经之前完整226帧1356数组/2712diag和literalfixtures精确等价；本次online生产者仍旧snapshot实现，新helper不改变这条actual轨迹。390/397suite是作者实际CPU验证，397pass20.75秒；评审个人范围另署。

图片cad_complete_baseline_actual_and_shadow.png作者已查看；两真实视频cad_complete_baseline_current_run/stereo_view.mp4与third_person.mp4。metadata同原完整goal，局部reference_length与SPL不用于新formal成绩。actual_goal_line_progress与metrics.goal_progress定义不同，报告采用冻结几何评估器沿线点积，不能混成同一个数。

## 下一必要层

本次没有把未知视场补成安全或靠历史shadow冒充成功，CAD过滤没有解决执行/35秒/连续位姿。下一项不再改网络或RGBD阈值；在这两条新prior-task完整日志上，**仅复用原冻结RollingOdometry理想gyro+encoder**进行whole-sequence适用域回归/姿态分量原因核验，原0.05/0.1米/2° gates保留，无fit/参数矩阵/控制动作。先检查该低速域是否符合旧预测假设，不能用旧Oracle失败宣布所有任务上不可用，也不能用新域通过救回旧全域拒绝；预测传感器白名单/标签后评，都是历史不是fresh。若新域失败则保留拒绝并按实际误差层确定下一项；若通过另冻结一个新完整在线只读序列，仍不直接移植控制。学习memory和正式E0/E1/E2仍受前置gate限制，没有新worldmodel成功。


Joint single fresh final review: C=0/I=0/M=1. Reviewer personally checked seven fixtures, inverse derivations, bounded hashes and full saved request/clock/prefix denominators; full source/map census, 397-suite, videos and timings inherited from author. Minor: rounding fixture lacks literal half-cell/negative-boundary cases; no claim that these boundaries were separately exercised. No runtime error identified; unchanged np.rint preserved. No second review or scientific rerun.
