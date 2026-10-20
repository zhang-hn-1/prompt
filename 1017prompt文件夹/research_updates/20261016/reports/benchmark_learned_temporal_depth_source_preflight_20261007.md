# 学习时序深度源：接口预检

还没有新深度评分或导航结果。本轮从已拒绝的重复性/连通记忆转向一个有明确理由的独立源候选；只选 MVSAnywhere Hero，不轮流跑网络。原始相机、光照、数据、控制与 35 秒规则保持冻结。

## 为什么选这个接口

[MVSAnywhere 官方论文页面](https://openaccess.thecvf.com/content/CVPR2025/html/Izquierdo_MVSAnywhere_Zero-Shot_Multi-View_Stereo_CVPR_2025_paper.html)和[官方代码](https://github.com/nianticlabs/mvsanywhere)给出多视角 RGB 与已知相机几何的深度估计方法。项目明确提供 Hero 权重及世界/相机矩阵约定。这使它成为当前时序、已知尺度诊断的一个候选；并不证明它能解决本机器人的灰色高光或遮挡。当前源码固定在 `5bd49bba0992aa56060e1aa3bb40b2150b04726f`，完整归档 SHA 和逐文件身份已保存。未修改上游源码。

[Stereo Anywhere](https://stereoanywhere.github.io/)研究双目匹配与单目深度先验融合，并针对非朗伯表面/弱纹理评估；它是问题背景资料，未在本轮下载或运行。[VGGT 官方论文](https://openaccess.thecvf.com/content/CVPR2025/html/Wang_VGGT_Visual_Geometry_Grounded_Transformer_CVPR_2025_paper.html)提供多视图几何预测，也只作参考。上述先验都不是对遮挡区域的实际观测，不得直接生成可信 free-space。

## 亲自读取源码得到的接口事实

1. Hero 使用 view-agnostic MLP，对源视图维做加权聚合；默认示例七源，实际实现的源维是动态的。两源推理仍需实际运行证明，不能凭注释声称已通过。
2. API 收 world-from-camera；USD camera 的 -Z 观察方向需右乘 `diag(1,-1,-1,1)` 变成光学 +Z。元数据“world”在核心卷积中实际为当前相机坐标，相对变换必须一致。
3. `BackprojectDepth` 使用像素索引 +.5，而本仿真使用整数像素中心。送入该模型的主点应为 (320,240)，原生输出评估继续用 (319.5,239.5)，不修改实际相机标定。K 为 4×4；官方入口内部匹配层缩放四倍。保留原 640×480。
4. 官方预处理会原位缩放传入 K，wrapper 必须传副本。原 RGB 不重画/增亮，不在此预检中引入新遮挡填充策略。
5. 默认入口没有 CAD token 掩码；`overall_mask` 是源几何范围支持，sky 输出也不是校准准确率。不能称模型内部所有注意力/特征都排除了机体，也不能把这些字段当成可信障碍观测。后续查询需显式保留原始 CAD 和两源投影支持，并标为学习估计假设。
6. 自动范围/细化在源码中可用；后续固定 .5–3 m 与一次细化，不用真值估尺度/裁深度范围，不扫其他设置。原准入数值门槛不变。
7. 初始化会动态下载 DINOv2，并加载匹配骨干；默认 checkpoint 加载存在宽泛 fallback。实际运行前必须固定依赖源码/权重并核验所有最终 state_dict 参数，不能静默使用部分权重或漂移的远程代码。

## 已完成的实际测试

四个 packet bridge 功能测试先出现四个 scaffold 失败，随后 **4 passed in 0.20s**。测试坐标中心/+Z、捕获数组不被修改、未来时刻在读取图像前拒绝、固定两条不同过去源/标定尺寸。

对六组冻结真实 RGB/knownpose/CAD 包完成 source-only 坐标复算；与旧独立 native 光学投影的像素差 <1e-12。当前及两个过去曝光时刻严格按源包，原 RGB 完全保留。未读 renderer GT、动作或未来位姿。`packet_bridge_audit_v1.json` 含输入 SHA、各包误差和依赖实测结果。

独立图复核发现 `>.999` 容差不是任意输入下严格的全部双线性 tap 未遮挡契约。另建未安装的严格 helper；最小 1/1024 遮挡贡献断言先实际失败，再改为 `==1` 且显式 FOV/有限性检查，**3 passed**。旧 ray/graph 预测和评分没有重写；独立复核证明旧六包严格检查零差异。此修复是新接口基础，不是恢复旧模型准入。

完整回归之前 489 项通过；新桥全套首次因下载的上游 `test_rmvd.py` 被收集而出现缺少上游包的 collection error，原日志保留。重新回归明确排除未安装的第三方源码目录，项目自身测试未排除；最终日志另存，待完成核验再追加实际数量，不冒称已通过。

## 未完成与下一实际入口

ball_wm 环境缺 lightning、kornia、antialiased-cnns、loguru（源码依赖已实际检查）；现有 Torch 不应降级，正式仿真环境不改。下一步在工作区隔离依赖与缓存，固定 DINOv2 和唯一 Hero 权重，先做两源原接口 smoke /完整参数一致性，再只跑冻结六包。禁止直接调用会漂移依赖的原 `torch.hub` 演示、未来 keyframe buffer 或真值尺度对齐。该问题可自行处理，不是需要用户审批的外部阻塞。

无 checkpoint、新模型推理、fresh-sequence、模型准确率、真实动作 A/B 或正式导航通过可以从这份接口预检推断。原来源/位姿/世界模型/导航拒绝仍有效。


## 接续：首次复核与因果入口修复（追加）

项目自身完整回归已实际 **493 passed in 23.92s**（明确排除第三方源码，首个 collection error 保留）。严格 checkpoint helper 三个测试实际 scaffold RED→GREEN，缺失/额外键及 shape 不匹配在任何参数被写入之前拒绝，不允许 silent fallback。隔离依赖安装成功并实际导入官方源：Torch 2.5.1+cu121、Lightning 2.2.5、Kornia 0.6.7；既有环境未降级，所有新增依赖位于实验目录。GPU 实测 RTX3060/12GB。此时仍未推理。

首次独立接口复核亲自运行十项小测试，全部通过，另查出两项 Important：原 draft 在完整时钟核验前解开捕获 payload，并从其自身时钟推导 now，且未核验 CAD 的帧/时钟；全输入 SHA 只在预测结束后封存。冻结实际数据没有发现未来混入，但原 wrapper 不满足强因果接口要求。原 draft 已单独保留。

作者另加独立 camera_clocks 当前索引/两过去索引、lazy payload 入口和 CAD/RGB/pose 时间/帧检查，四项功能测试实际 RED→GREEN。六组真实数据每组 16 个桥接字段与原正确输入一致，分别给当前 RGB、过去 pose、CAD 注入时间异常，共 18 次在任何图像/位姿/遮挡 payload 读取前拒绝。新增 separate custody sealer 在首次推理前核验并封存五个曝光的 15 个捕获文件、独立时钟索引、冻结代码/设置/权重/源码身份；模型每次仍只解开当时及过去两源。数据保管时全文件身份封存不等于给当前模型未来图像。旧输入/历史评分保持不变。

修复后完整项目回归 **500 passed in 24.54s**；排除整个隔离第三方源码/依赖目录，项目自己的测试仍全部包含。独立原报告在 `reviewer_mvs_source_adapter_v1/review_report.txt`；修复和真实异常注入是作者亲自核验，没有伪称进行第二轮复核。源码及输入桥具备有界测试证据，但实际两源模型/准确率/物理在线接口尚未通过。

唯一 Hero（官方 2.42GB）下载仍在进行。接续 driver 将等待相同权重与上述复核闭合标记，然后依次运行 whole-source sealer、唯一六包 predictor、封存后 assessor；每个计算进程 600 秒，准备阶段 2100 秒。原下载有 900 秒行政时限，仅允许一次同 URL/同字节 HTTP Range 续传，不更换权重/模型/参数；失败原日志与 partial 保留，不做静默科学重跑。不存在正式导航通过或持续计算承诺。
OldprefixSHA c5adaea10685bcdd4c6b440a846248c327804582a438cda6fcab258783607f2c


2026-10-07T03:55:31.804457+00:00 evidence count correction (append-only): Actual capture_causality_actual_audit_v1.json reports 13 identical bridge fields per case, 6×13=78 fields, not the earlier prose 16/96. All18guardtests andall500suite pass unchanged; typo in prose only, source outputs notrescored.
OldprefixSHA 6fdb8033e8398f669644313e8f72fcfc32d2f672f54c67f483bf329184bba601


## 2026-10-07T04:01:17.079690+00:00 single official native-output API recovery
Original uniqueHero acquisition completed848.67s/2420601860bytes SHA00878bab903b1d384cd6bb9ea092e247a9036d429cb8bcc3d6dcbb81f67e556d; no Range resume used. FirstGPU235L ran but480x640outputassertion failed, originalstatusfailed/logs/runtime557stateexact/seal preserved, no pointpackets orGTscoring. Checkpointprediction_scale=.875; directadapteromittedofficialload_model_inference decoder.set_prediction_scale(YAML1.0). No actualfirstGPUshape recorded, do not fabricate; realCPUofficialDPT fixture28x42 vsnative32x48 reproducedRED1thenGREEN1withallweights unchanged. This is one explicit administrative API-fidelity fix frozennative640480, notresolution/threshold/architecture/NN rescue. Newindependentv2sealer/predictor/assessor/outputnamespace preservev1bytes. Sameckpt/source/DINO2/camera/RGB/poses/float32/refine1/.5..3/originalgates; full501PASS24.94s. Secondexactstatecheckafterupscalecall+postresultold/newweightdigestplanned; nofurtherretry ifv2fails. Recoverydriver run_mvs_hero_native_api_recovery_v2.py abouttolaunch ONEGPUrun onfixed6. Checkitsstatus/pid/logsfirst, notoldv1failure/stalecontinuation. WholeNOTcomplete/formal35sE0E1E2NOTtriggered; pipeline stillsourceknownpose/development notrealnav/nativepose/worldmodeladmission.
OldprefixSHA ea857cbb3fa367da17900ef89b88fa7f13d42d90ae1571978aa914ea8ee804cd
