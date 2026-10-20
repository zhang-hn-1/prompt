# 被拒绝稠密来源的唯一whole-sequence空间消费诊断

只复用新E2yaw−20完整65帧、已固定B0/adapter深度，模型均REJECT。SameRGBoutputs projected with CURRENTknownpose，2models×current/history=260maps，不重复推理/采集/fit，不改source/mask/PID/rig/light/35s或原门槛。本序列已用于解释设计，不能再称freshheldout。

纯consumer白名单：depth_m/confidence/nativepositiveD、frame/time、left/right_camera_usd_world/currentrootposition/time，bodymask固定。不能读GTdepth/occupancy、动作/未来state。点stride2，每眼source记录真实currentframe+leftpixelid/rightnearestpixelid/t；d>1，.5<=depth<=3、finite/confidence+mask以及partnerinFOV/mask才记录。knownpose转换USD→optical diag1,−1,−1；XYgrid原点FIRSTrootxy−8，shape161×161/.1，extent不输入地图内容。source模型未准入，不承诺free/action。

sigma取±1px扰动造成的最大depth差×raylength，仅敏感性非对应正确/物理bound。candidate point到当前已知root>=.55m，避免残余自身端点；真实labels不能筛点。PointSupportMap原样：world-Z floor0，abs(z)+sigma+pose_sigma0<=.05且total<=.05，hazard .05<z<=1.5；TTL10s/来源不刷新/未来不读/atomic，floor endpoint不插值/扩展/raycarve/脚下seed。所有unclassified/outside记录计数保留。只保存每currentpacket点NPZ+query/source provenance一次及maps，snapshot不重复写全部records JSON，record摘要来源时间范围/计数+原packet可重建。

current仅同帧，history仅<=t且10s来源；unknown/current/history/proxy hazards分开。每t使用当前knownrootxy，检查旧same.7radius+grid半对角的当前disk支持，任何unknown/hazard阻断；仍full3Dcertified/actionauthorized=false，无运动计划/在线导航shadow。本consumer回放是archive诊断不是实时runtime。

先封存源码/模型outputs/knownpose-only包，全部260maps/130packets存manifest后才labels。offline评估沿每源query actualrenderer深度+已知pose，计算估计floor记录错分类/三维错误/历史污染与所属格冲突。不把2Dcell或ray代理当物理collision。真实完整轨迹6→32s sweep仅在offline评估器读取已存states计算，不能传回consumer或称当前可预测trajectory；评估每已存地图该固定空间集合支持，分开currentfootprint与全空间reference。

门槛：消费者输入/因果/身份/重建通过，只证明接口；floor0错误代理(不是安全保证)、当前disk及真实固定完整trajectory sweep均要求unknown0/hazard0的必要支持条件。严格coldstart只评6s运动前map，当时history用既有0..6RGB，不事后灌历史。source候选原REJECT不倒改，即使2.5D支持通过也不会控制准入/3D/nav；失败关闭此唯一消费诊断，不再换mapper/TTL/radius/模型。直接明确缺失信息与下一可执行任务或须用户解决的冷启动条件，而不是继续同类审计。

Ruling: 整体自主授权，无逐阶段审批/删除旧字节/Gitcommit。既有PointSupportMap复用，不重写planner/控制。taskprior/safe-start只可另标签，不能污染原cold-start或冒充observedfree。尚未运行RGBestimatedpose/世界模型/formalE0E1E2。
