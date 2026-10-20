# 双目来源事件交接实测与单眼外观范围验证

2026-10-07T08:16:12.728203+00:00

本轮两条实际物理对照已关闭，整体导航研究尚未完成，原35秒正式E0/E1/E2未触发。只有控制消费界面改动，不改相机、光照、roll PID、映射、规划器、原5次低速确认/速度反弹清零/速度策略/10s TTL。已有所有拒绝结论保留。

## 实际同版本对照

同一新runner、同一2.6m局部development任务、同一起点、同一35秒预算、已知位姿特权。B0固定18s窗口；C1连续两次2Hz当前来源/5个10Hz低速/100hazard/2格确认事件，确认当前packet出生，历史事件保持锁存且handoff仍需当前有效来源和低速，不刷新几何出生。

| 项目 | B0 | C1 |
|---|---:|---:|
|真实交接时间(s)|17.999999597668648|14.999999664723873|
|结束时间(s)|35.0|26.5|
|终止原因|timeout|collision|
|真实最终goal距离(m)|3.356047018665342|2.0280522574433606|
|goal距离减少(m)|-0.7560465835514294|0.571948177670552|
|真实沿目标轴进度(m)|-0.7351109564304354|0.5844247758388517|
|接触采样数|0|5|
|真实运动路径(m)|2.5577228182521443|2.1697889838212125|

C1确认145/150，于15s真正改变请求和applied；提前3s并非成功，26.5s碰撞，拒绝。B035s超时且偏离目标。两者都未进入.2m目标范围。C1准确源150为1618/1618正确，之后9个非空当前/旧源评分全部≥95%；但只有150/155/160/165四个非空qualified-current更新，随后没有新支持。B0全非空源95%门槛不通过（200:91.11%、210:84.49%、295:82.14%等）。不能以空输出证明可靠覆盖。

## 接口与执行证据

- B0: 71次源更新/2627字段复建，142未来clock payload拒绝，39次截断前缀复建；350个最终10D请求精确复建；1750实际dispatch ticks、1745个实测反馈clone ticks、349快照一致。首5ticks未clone，不是递归物理预测。
- C1: 54次源更新/1998字段复建，108未来clock payload拒绝，33次截断前缀复建；265个最终10D请求精确复建；1325实际dispatch ticks、1320个实测反馈clone ticks、264快照一致。首5ticks未clone，不是递归物理预测。

不二次调用policy；top-level request仍是旧nominal诊断字典，最终动作读取local.request与policy_inputs。双臂各10个未来事件clock陷阱通过；仅这些接口及前缀范围，不声称全archive future文件poison。B0/C1 collector分别4202/3488 identity checks、1562/1188 packet-mask fields、2100/1590 maps、4200/3180只读shadow fields精确；曝光时间delta均0。真实两路视频B0各351帧、C1各266帧、10fps实际解码。整个shadow-on/off物理episode未单独运行，不称已通过。544项CPU回归是作者launch前实际运行25.20s；第一独立skillreview C0/I0/M1仅该次范围，作者修driver失败topstage后launch，旧bytes保留；未由reviewer运行物理或GT。

## 因果范围

分叉前实际physical states、joints/proprio/action histories、execution snapshots、已知camera poses直到frame150一致。最终request首差异150。但RGB从frame0不同；nominal/tracker从85不同。因此同版本同初始条件有效，但不构成完整严格单因素源prefix相同A/B，不能将两条最终行为差异全归交接时间。scene/runtime/controller/policy源码/rig/light/source SHA身份一致，实际类PriorRGBPolicy及roll_pid BankRuntime，未以B0名称替代核验。

## 具体失败层次

C1正常沿live参考跟踪：交接后独立投影peak .076327m；B0 .167044m。C1交接首帧参考已穿过离线实体占用proxy，suffix-clearance .043860m，而当时观察点screen .635153m。C1规划扫掠未知占比最低99.5717%，并非已观测free；frame150为466/467未知。

C1在18s源出生仅3s时实际hazard map已空，原准确seed仍可保留3格，25s才TTL失效；26.44s首接触。但保留seed的事后叠层115帧仍会让穿过proxy实体的参考通过。B0对应165帧。故不能仅延长TTL/NCC或提前启动解决；“不碰已知几个点”不等于避开完整障碍。上述proxy只事后因果诊断，未进入policy/新的counterfactual物理标签。观测点必要clearance通过（B0 min1.3223/C1 min1.5331）不能冒充完整安全。

接续十个预定事件曝光的全native GT ownership：frame150碰撞相关.6m表面左眼70314像素，右FOV外59735/CAD挡10579，共同表面0；145同样0；180共有1603、模型有效1。245只有左眼1081相关像素但都右FOV外，250/260/265相关域0。区域由后来真实碰撞位置选定，严格只事后说明，不是在线可用区域。不是只把旧日志重放称在线；B0/C1是两次新物理采集，此GTownership是采集关闭后的诊断。

## 紧接着完成的外观与空间歧义实验

没有继续调PID或换深度网络。唯一低阶外观候选只从合格源150左右RGB像素得到Lab median/MAD，尺度floor8、1.4826MAD、三维距离<=3，10个过去完整曝光mask先seal后GT；10future-payload traps通过。所有假设在15s出生，不将过去RGB回填成过去已有depth/occupancy。全部是development，不是fresh。

|frame|eye|外观精度|可见hazard覆盖|
|---:|---|---:|---:|
|0|left|0.9987|0.9540|
|0|right|0.9891|0.9209|
|60|left|1.0000|0.9420|
|60|right|0.9993|0.9139|
|120|left|0.9998|0.9599|
|120|right|0.9983|0.9659|
|145|left|0.9996|0.9662|
|145|right|0.9991|0.9600|
|150|left|0.9981|0.9663|
|150|right|0.9983|0.9619|

PASS_APPEARANCE_CLASS_ONLY，不能代表深度、空间占用、free或导航。紧接唯一静态视觉锥几何歧义集合：过去10个外观mask/poses、.1mROI格心、7x7完整CAD-free、至少2曝光/90%视图相容，42336格保留9484格，生成seal后评分，未读GT拟合。真实表面覆盖91.75–97.996%，但最近假设表面精度仅1.31–6.69%、p90距离1.212–2.221m，REJECT_APPEARANCE_CONE_GEOMETRY。大量相容体积不等于真实实体；未用体积排除来证free，未接入planner，未改ROI/pitch/支持阈值救分。

随后继续冻结15s外观参数对155..265全部23曝光/46眼历史motion回归：44真实hazard>=100练习、2未练习；首次失败240，最终REJECT_LATER_MOTION_APPEARANCE。GT>=100而输出<100必须失败，不因空预测隐去失败。原seal绑定plan旧prefix，评分前补充分母说明保留旧bytes SHA，未看GT改标准。运动后拒绝不能被前10例通过覆盖。

只做一次分层原因：所有失败输出中，GT不足0，距root<=.55为260726，物理height-hazard但轴向范围外1170894，原域正确380466；距root近不能直接证明机器人自身，也可能已接近障碍，不能直接称body-mask bug。近距离外观识别与原.5–3m几何测量域不同；本轮保留原口径拒绝，没有改标签救分。下一需要真实表面/robot prim所有权与近距离bearing消费接口，区分真实近障碍、机体误识别和无可靠距离，随后才可有控制候选。

## 实际产物与接续

- 真实轨迹：[图](../experiments/stereo_memory_navigation_20261004_v1/source_event_handoff_actual_v1.png)；作者实际查看。
- 外观10例：[图](../experiments/stereo_memory_navigation_20261004_v1/qualified_appearance_extent_actual_v1.png)；作者实际查看，粉色为hypothesis不是depth。
- [C1双目视频](../experiments/stereo_memory_navigation_20261004_v1/source_event_handoff_C1_run_v1/stereo_view.mp4) / [第三视角](../experiments/stereo_memory_navigation_20261004_v1/source_event_handoff_C1_run_v1/third_person.mp4)
- actual stage、protocol、postclose seals、全部source/request/physics/map/contact/video、四类个人核验、全部GT scores及sealed appearance/cone/motion predictions保留于实验目录。

行政失败：coverage audit V1 numpy scalar frame导致unhashable，日志/code保留，V2仅scalar item()规范化，原方法/阈值不变。未删除原实验，未提交Git。正式导航、RGB估计位姿、fresh跨完整序列和在线新的外观consumer仍未完成；不能把本报告称整个项目完成。
