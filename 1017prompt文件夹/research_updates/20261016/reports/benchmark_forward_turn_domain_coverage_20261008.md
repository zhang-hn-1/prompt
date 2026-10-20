# 当前低速转向域补测与三个点来源消费者的边界

唯一72秒实际序列完成，三个事先固定尾窗通过必要稳态一致性：**PASS_THREE_CURRENT_SEQUENCE_POINTS_ONLY**。独立三个点primitive来源消费者在同一历史地图的唯一CPU preflight无路，已拒绝，未触发新导航仿真。完整BankMap、无未来反馈瞬态预测、视觉导航和原35秒benchmark均未由此通过。

原gains .4/.1/.7、mainaccel .5、auxslew30°/s、aux速度supervisor.3、固定相机/照明保持。一条完整reset之后连续raw请求：0–6零；6–26主轴+.5/aux0；26–46+.5/aux+30°（bank+22.5°）；46–66+.5/aux−30°（bank−22.5°）；66–72零。没有内部reset/smoother清空、policy/planner动作、模型fit或参数扫描。此72秒开放平面诊断不改变原35秒导航规则。

固定尾窗21–25.5、41–45.5、61–65.5，按原producer31样本中心XY几何测量，离线标签不作在线模型输入。每窗225/225有效，速度方向匹配1。

| 请求bank | 实际bank中位 | XY速度m/s | body曲率1/m | 曲率IQR | bank95–5跨度 |
|---|---:|---:|---:|---:|---:|
| 0° | .287356° | .203881 | −.009525 | .029082 | .089159° |
| +22.5° | 23.688439° | .161111 | −.784383 | .033211 | .372956° |
| −22.5° | −21.862461° | .187164 | .800764 | .052727 | .620632° |

全部通过原valid>=.9/IQR<=max(.2,.3absmedian)/bank跨度<=3/方向>=.9。相对旧同方向同主轴+.5点，曲率误差.023438/.042842/.004161，速度差.001987/.027438/.002351m/s，分别满足冻结必要一致性限。正bank速度差接近.03上限，不能无误差替代速度。旧gain为.8/.1/.35且其他身份差异保持，不是gain因果A/B或全域认证。

实际bank按距目标2°连续五个10Hz到达：直线6.4秒；正bank28.5秒（请求后2.5秒）；反转负bank50.8秒（请求后4.8秒）。图上转向切换后的XY几何曲率仍振荡，到角度不等于曲率已经稳定。66秒零请求后，实际停稳资格68.5秒；停车尾速度peak.169819m/s、位移peak.213381m。没有用关节过零替代世界XY五次低速/反弹清零规则，也不推导任意短时停止模型。

实际UTC16:12:31.132–16:16:24.131、exit0、wall232.999秒，最多1条、无重跑。protocol SHA080b96c0d686624120eec8147f8525bf7cc3fa8213851befd9b2d8affa4d8a50；175source/14217input实际身份通过、1597postclose文件SHA；720请求/3601PHY/721capture/145RGB包，两个721帧10fps视频；4321available contact记录全0。作者3600ticks实测反馈条件smoothed/applied精确重建。初始编号execution000为null，独立initial_execution_after_reset保存完整初态；完整编号720+独立1=721个可用snapshot，不把null算完整。没有递归物理预测或未来feedback actor。

行政验证失败v1沿用短序列累计时间与double .02的1e−6门槛，72秒浮点dt累积差达1.609e−6。核验官方runtime实际float32 .02时基与全部3601样本**逐元素精确相同**；独立v2按同一真实时基和capture/physics对应关系精确核验，没有放宽gate、改原数据或重跑。失败源码/日志、评估源身份保存。

![作者实际查看的完整响应](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/forward_turn_domain_actual_v1.png)

## 单一来源消费者preflight为何拒绝

只允许三个当前前进点+.5/0,±30编码aux，几何curvature复用旧冻结BankMap−.032964/+ .796603/−.827225（新实测只验证，不fit）；未知点返回None，不插值、外推或直接移植全表。原Hybrid A*仅增加来源callback、cost/rootcost/.3primitive/3000nodes/raster .870711/guard.6保持。feature-off原search输出自一致；5个CPU测试1.47秒，原V1实行为RED1fail3pass后4GREEN，后路径设置失败日志保留。

唯一历史R1frame140原MODELmap/estimatedpose/goal查询返回unreachable、expanded2，无有效waypoint。固定3个forward .3m首段都进入原EDT .806226<.870711，不能前行；原baseline需要先倒车退出该区域。preflight拒绝保留，未缩半径、扩大node预算、增加角度或跑GPU。范围缺口是当前−.3倒车分支，已有72秒前进数据无法填补；同时4.8秒转向反转响应尚非可忽略。

该版forward-only来源候选不作为可用模型，不在失败后补reverse点偷偷改成通过。下一阶段如补必要倒车，应独立冻结具体单点、完整序列开发/验证、独立扩展支持字典和新的准入；不得推翻本版拒绝或把旧seen数据当fresh。

证据H下forward_turn_domain_{protocol,status,assessment}_v1.json、verification_v2.json、evaluation_identity_v1.json、forward_turn_domain_run_v1/{physics,requests,initial_execution_after_reset,history,frames,execution,feedback,post_close_manifest}.json/npz及双视频；supported_forward_point_preflight_v1/{assessment,first_edge_cause,actor_pretruth_seal}.json。作者实际运行、核验并查看图；primitive源码的首次独立review在另列记录，不把reviewer读到的作者证据当个人GPU运行。研究整体、学习增量、严格视觉及原E0/E1/E2仍未完成。
