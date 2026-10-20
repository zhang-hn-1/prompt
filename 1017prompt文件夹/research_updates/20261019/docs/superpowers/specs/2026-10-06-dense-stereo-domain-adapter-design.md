# 原双目稠密来源的唯一受控域适配

范围：接续已拒绝的原Fast、稀疏特征及运动关键帧来源；只验证原Fast-FoundationStereo相同checkpoint的监督域适配，不换网络/相机/光照、不改变原90%/99%/.1m、.5..3m与1.5px左右一致性，不做控制/世界模型准入。此候选保留RGB双目代价体，不能把learned depth当已证明真实观测或自由体积。

原始代码forward(test_mode=False)返回初始视差和迭代视差；固定feature/cost-volume/context及BN统计，只训练update_block、spx_2_gru、spx_gru命名参数。先ONE旧E1帧前向/反向可行性探针，无optimizer/保存权重/候选选择，失败则拒绝，不换结构。通过后ONE300 attempted AdamW updates、lr1e-5、weight_decay1e-4、batch1、8iters、原full-resolution rectification/max_disp416、GradScaler、norm clip1，fixedseed0。最终第300attempt权重唯一候选，不以验证或最佳训练checkpoint挑选；非finite loss/grad则拒绝，不重新fit。

fit来源只旧完整E1原B0：0/28/56/84/112/140/168/196/224/252/280/307共12pairs；旧hard0004归档0/10/20/23共4pairs。两眼共32固定输入，round-robin300，无augmentation/随机帧拆分。标签仅fit监督：renderer axial depth z→f*.6/z、.5..3、0<d<416、原双mask和partner visibility |zr-z|<=.03+.01z，无future或动作标签。右眼flip/swap后同positive disparity。loss eight upsampled predictions weighted gamma.9 normalized weighted mean smoothL1beta1，invalid不参与；空标签input拒绝。不读取位姿为推断特征。不训练额外分类/残差头。

fresh：ONE新E2 yaw−20°整32s序列，原zero6→main+.5/rawaux−30°20→zero6；仅视觉验证数据采集，不重标定/导航。起点一次reset，内部无reset；相机/同步/光照/rollPID/速度supervisor/commands原值，save65actual RGBpairs/321capture/1601physics/视频。全部raw/source先于fit封存；不得看fresh模型成绩改模型、budget、数据或gate。不同初始条件完整隔离，但同E2场景不是fresh环境泛化。

predict接口只left/right RGB、原body masks；samePadder/forward迭代、mirrored right/冻结bilateral。GT真值只在所有65pair×2model(B0/adapter)预测manifest保存后评分。不能用maskconfidence清理错误救分。原独立body/floor, overlap/all分母定义复用audit_stereo。主来源gate body_overlap: eachframe>=100有效query时完整correct>=90%、输出precision>=99%、finitep90<=.1m；pooled>=100且同门槛，数据不足如实记录，不称通过完整域。floor overlap与近场扫掠支持单独报告，body通过不代表floor/盲区支持。保留B0对照旧失败及本次fresh结果，candidateB0保存SHA。

拒绝一次即关闭该模型域适配，不换LR/iters/层/网络/预算/新fit。如失败做已有预测+训练曲线一次有界原因核验，明确来源与rig近场不可见问题不同；不把小stage终点当整个研究完成。通过也只能回到来源记忆knownpose验证；estimatedpose与真正action-conditioned世界模型后续独立准入。

官方资料个人本次打开 https://github.com/NVlabs/Fast-FoundationStereo 和 https://docs.nvidia.com/tao/tao-toolkit/latest/text/cv_finetuning/pytorch/depth_estimation/fast_foundation_stereo.html 。本地研究checkpoint带pruned模块，未采用TAO另模型或新依赖；我们的训练入口由当前源码核验，不把官网模型效果当本项目效果。

Ruling：用户完整独立推进授权优先于skill逐阶段审批/commit/cleanup。保留全部旧source/data/结果，独立命名候选；作者独立inline执行，一次最终readonly审查。spec自查已完成：一个网络/一个拟合/一个fresh whole序列，梯度探针不保存参数且不选择hyperparam；old heldout不称fresh。
