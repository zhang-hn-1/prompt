# 当前低速前进转向域：唯一必要覆盖序列

目的来自实际视觉闭环R1：同版本请求链125/125精确；early正常tracking多次要求曲率1.2–2.2，而旧低档BankMap支持上端.796603，当前匹配singleleft22.5稳态.792373。精确lookahead到255才改变protocol，此前真误差peak.321407>.2，故唯一候选已拒绝，不跑无望fixed-reference pair、不调距离参数。

当前仅主轴+.5/负bank−22.5冷起步20秒稳态受20261005匹配实测支持，无法推出正bank、直线或移动反转转换都有效。唯一新序列补这一具体缺口，不重做矩阵、不拟合网络/换模型/PID/scan，不授权整张BankMap或动态预测模型。

冻结一条完整72秒开放平面诊断，完整reset只在开始；0–6 raw0；6–26 main+.5/rawaux0；26–46 main+.5/rawaux+30°（bank+22.5°）；46–66 main+.5/rawaux−30°（bank−22.5°）；66–72 raw0。保持当前gain .4/.1/.7、mainaccel.5、auxslew30°/s、aux速度supervisor.3、rig/light/renderer同步。无policy/planner/Oracle act，没有新导航成功率；原35仍不触发。geometry世界XY曲率按原producer31点中心窗口测量，明确离线不是预测输入。

事前固定尾窗口21–25.5、41–45.5、61–65.5（从对应开始15–19.5秒），不挑窗口。复用必要稳态门槛validfraction>=.9/曲率IQR<=max(.2,.3absmedian)/bank95–5跨度<=3°/前后速度投影方向>=.9；与冻结同方向BankMap low对应点稳态误差<=max(.2,.3absoldk)、速度差<=.03。旧表gain身份不同保留，只准入这三个当前顺序/请求点的必要匹配，不叫全域、新held-out模型验证或因果PID增益实验。转向切换实际bank到达/XY曲率变化/停止五低速度/反弹与位移另测，不能用目标变化时间替代物理反馈。

最多实际1条。原始全部状态/请求/feedback/源RGB深度/时间/视频与代码配置SHA保存，依旧是开放平面特权诊断，不纯RGB导航或source修复。失败后不增加保持时间、角度、PID或再跑另一序列救门槛。结果下一项只能针对具体规划需求与实测范围接口，不能把该结果直接替换planner或清除历史导航/学习/来源/pose拒绝。
