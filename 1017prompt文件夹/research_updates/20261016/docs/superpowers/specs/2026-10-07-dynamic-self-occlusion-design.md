# 当前关节条件下的已知机器人自遮挡：唯一CAD候选

问题证据：前轮两个fresh阻断格12点实际在距root约.396m表面，三维误差约1.12m；旧mask只覆盖base_link。原完整目标35s current仍失败，同源shadow已完成但无替代真实轨迹。固定相机/灯光/base_link mask/旧gate不改。

只验证一个候选：用已知机器人CAD可见三角面和当前proprio17维中三关节位置9:12，按UsdPhysics joint局部坐标FK将base_link/link1/link2/link3投影到两个固定camera optical frame，再用原3pixel膨胀与冻结base mask取并集。不使用GTdepth、实例ID、场景几何、未来角度或当前/未来执行指令决定mask；不换网络/重训/改变planner/PID/速度。原mask独立保留，候选版本另立。

第一步复用已有四帧（local history0/135与fullgoalcurrent200/350）离线仅投影，验证base-only与冻结mask精确相同，当前CAD mask是否遮住已识别12点；这是development，不是fresh准入。

第二步必要时ONE12s受控运动诊断、最多4个验证captured时刻0/3/6/12。序列完整reset一次，内部不重置物理/PID/smoother；请求序列事前固定，不用未来真值。记录真实joint_names、当前三关节、base/camera与三个link pose用于FK独立核验；渲染instance-ID只验证自身可见像素身份，保存offline_namespace不进候选；网络不拟合、不推理也不控制。完整捕获/曝光时间一致并断言render不推进物理。

先冻结接受标准：base mask身份exact；四时刻FK位置误差<=.001m、旋转<=.1deg，相机rig相对base误差<=.001m/.1deg；不读未来/真值的接口test pass。若instance annotator提供确切robotmesh像素，候选mask对真实自遮挡覆盖>=99%，新增mask误屏蔽非自身的额外像素<=3%全图（原base mask单独统计、不倒改原结论）。这只是selfmask几何准入，不等于双目完整来源/地面覆盖/动态导航/35s通过。实例ID不可用须明示未验证，不能用CAD自证同一CADmask准确；FK不通过只做有界原因定位，不能调参救门槛。

通过后才在保持网络raw预测/旧confidence基础上比较候选的source suppression和有来源记忆；不以事后真值选点。只在局部A/B充分证据后移植完整goal，不把只读投影称动作改变。若失败拒绝唯一候选，不另换网络/扩标定矩阵。wholeNOTcomplete。

封存前的执行说明：底层仍按10Hz捕获121张RGB用于完整12s视频和传感器同步，只有0/3/6/12四时刻保存FK/实例ID验证records。自身mesh可加诊断semantic标签使renderer ID具有robot_self身份；标签不改变几何/材质/可见性、不进入固定请求程序或候选mask。装载的是已批准E1起态，3s前零请求；3–6s main+.3/aux+.15，6–9s main+.3/aux−.15，9–12s零，序列内不重置。并非E1导航运行，不用12s结果替代原35s规则。旧四development投影均完成，特定12点被CAD候选全部遮住，仅为设计/开发支持，不是fresh gate。
