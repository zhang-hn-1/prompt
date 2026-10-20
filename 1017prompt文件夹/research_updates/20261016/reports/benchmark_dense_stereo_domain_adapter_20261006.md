# 原双目深度网络的单次监督域适配与独立完整序列验证

唯一实际训练候选拒绝来源准入。完整双眼可见障碍高度带正确率从24.7406%增加到30.7991%，但保留precision从99.9633%降到97.6839%，远不足完整90%且跌破99%门槛。25个有>=100标签的验证帧，两臂均0帧通过；其余40帧数据不足。地面和近场仍无充分支持。没有接入控制、改变相机/光照，亦无新导航成功率。

## 身份和唯一模型

采用此前原[官方Fast-FoundationStereo](https://github.com/NVlabs/Fast-FoundationStereo)研究checkpoint 23-36-37，原源码commit476f4249561f7c79ca707326954f9255643412a6；原checkpointSHA af0658f289ec840b292645f8d5538978f06e8cabaa1fd31e84acc91af268e990。本次个人打开官方repo与[TAO训练说明](https://docs.nvidia.com/tao/tao-toolkit/latest/text/cv_finetuning/pytorch/depth_estimation/fast_foundation_stereo.html)，核验当前本地forward训练返回与真实gradient入口；没有采用另一TAO模型/源码或升级环境，不将官网其他硬件结果当本机成绩。具体checkpoint实际17654857参数，不能用通用modelcard14.6M替代实际身份。

feature/cost-volume/context、所有非decoder参数和running buffer保持冻结eval；只训练update_block/spx_2_gru/spx_gru共845814参数，52参数张量真正改变，670其余state项在实际训练中逐个散列完全相同。保持真实双目代价体、8iterations、full640×480原生图逆时针90°、measured眼别、416span及原1.5px双向一致性/两眼mask，不增加分类或残差头。源预测仅两个RGB+固定bodymask，无pose/depth标签/动作输入；学习测距仍不能自动当成认证观测自由体积或世界模型。

## 前置失败、一次数据补正与时间顺序

首无optimizer梯度probe因官方torch.compile缺C compiler，在loss/backward前失败；原log/protocol保留。提前独立execution addendum声明TORCH_COMPILE_DISABLE=1，同B0和candidate均eager原Torch算子，没有替换模型函数。唯一基础设施重试20:25:36.422878→39.725154UTC成功，31977监督像素、loss.94461584、全部trainable梯度有限、all state unchanged、0updates，峰GPU5579253248bytes。它只核验可训练入口，不是模型成绩。

原32-input训练清单由完整OLD E1 B0十二帧0/28/56/84/112/140/168/196/224/252/280/307及OLDhard0004四帧0/10/20/23×两眼组成。V1 preparation在E1 168仅42有效标签时因代码额外>100断言停止，尚未建立optimizer/loss/backward/保存学习权重。逐OLD输入核验发现6空：E1 196两眼、hard20/23两眼。原32固定设计的完整数据资格拒绝保留；原running行政状态因try没有覆盖preparation未更新，用独立terminal_audit解释，不覆写原字节。

在任何fresh模型成绩或标签解码前，只做一次明确data-provision addendum：拒绝这6个空输入，26个非空旧输入按原序round-robin，39..71像素低样本眼仍保留，不删困难眼或选择最佳训练帧。保持同唯一decoder结构、lr/budget/验收门槛；正式评价>=100仍不变。没有第二个实际拟合/网络/超参候选，原32设计不伪装兑现。新协议46source/132input预封存，其中原40source/131input和旧失败协议保持可追溯；此裁决成本是监督覆盖只限这些旧可见区域，不能认证全域或fresh环境泛化。

## 新独立完整验证采集

ONE新E2 yaw−20°完整32s序列，实际20:26:01.858508→20:28:03.620250UTC退出0，早于拟合。只改变验证起始yaw/独立namespace；原zero6s→main+.5/rawaux−30°20s→zero6s，rollPID(.4,.1,.7)/mainaccel.5/auxslew30°s/auxvelocity.3不变。原相机ball_stereo_front020_up005_v2 X.20/Y±.30/Z.05、640×480/f584.888889和fixed_camera_fill_v1不变。原scene c3514039631ac3b22d035cd02b6f7fdc44b2dd2e8a85afc1bf9a5c122ef1522d，occupancy53b7341d727c9c9dfd2b2aa49784fa3afca20638eea497d261ce6ea2f2347883。

320请求/321capture/1601physics/65actualRGBpairs，reset一次、内部无reset；capture使用fixed3→singlebasecapture→fixed3→imagesonly，同步两眼render时间差均<=1e−5。717raw/feedback/identity/video产物及65RGB-only NPZ于fit前整体封存；预测包只有left_rgb/right_rgb/time_s，truth保留独立raw，封存只读opaque bytes。作者核验782采集SHA、全部321capture时序、两视频各321帧10fps逐帧真解码，0/160/320拼图已看。1921contact记录可用，不把它们的数量当碰撞次数；本采集不是导航成功、运动重标定或控制A/B。同E2环境不同初态whole-sequence独立隔离，非fresh环境。

## 实际300步训练与完整双臂预测

唯一实际fit20:32:15.247235→20:34:56.653981UTC，300attempt=300optimizerupdates，158.568s，无nonfinite梯度/loss。固定seed0/AdamW lr1e−5、weight_decay1e−4、batch1、normclip1；GradScaler initial1/growth1000在fit前声明，300步不改scale。不选择bestcheckpoint，只保存最终第300步decoder权重SHA df9ff024e3fcc62461fa46a256e8af9048e673ec91103481e79a5d02b1cdd37b。旧label以axial z→f*.6/z，.5..3/正视差<416、两眼mask/partner可见一致|zr−z|<=.03+.01z约束，仅用于fit。right eye flip/swap后同正视差；loss八个upsampled预测的gamma.9归一smoothL1 beta1，空/invalid不监督。主source gate不参与拟合。

26个相同OLD输入首次loss中位22.67685→末次7.66899，25/26降低；不同example样本量和难度变化，不能把整条波动曲线当泛化性能。新candidate不是任务6的action-conditioned世界模型，也不是学习记忆网络。

实际两臂预测20:34:57.256275→20:36:57.417097UTC，全部65pair×B0/adapter=130输出在20:36:56.825058UTC manifest完成后，20:36:57以后才评价fresh标签。candidate只加载命名decoder tensor、shape/schema核验/weights_only=True；其他部分仍从原checkpoint加载。B0和candidate同eager双向，原confidence与mask完全不变。本机双向source p50/p95 B0 .8355/.8499s，adapter .8297/.8435s，不能称10Hz端到端实时；历史编译路径性能不倒改。

## 完整分母、覆盖与错误

沿用audit_stereo独立GT几何：axial.5..3m、距root>=.55m、冻结selfmask、高度body.05..1.5/floor|z|<=.05，actualpartner in-FOV/selfmask/遮挡depth一致与stride2。三维误差=depth差×ray长度；缺失预测留在全query分母，不把finite p90当全域精度。

|验证子集|全query|B0≤.1m|candidate≤.1m|B0完整正确率|candidate完整正确率|B0保留precision|candidate保留precision|candidate finite p90(m)|
|---|---:|---:|---:|---:|---:|---:|---:|---:|
|body_overlap|109961|27205|33867|24.7406%|30.7991%|99.9633%|97.6839%|.0558421|
|body_all|674893|27348|34075|4.0522%|5.0490%|99.2488%|97.3961%|.0573106|
|floor_overlap|225|0|7|0%|3.1111%|null|100%|.0794819|
|floor_all|20433|0|20|0%|.0979%|0%|4.8780%|.8796166|

body_overlap保留B0 27215/candidate34670；wrong10→803，新增输出不是全部可靠。floor_overlap只有7个正确保留点，不能承载地面连续面/3D扫掠支持，floor_all的390个错误保留也不能写free。相同query类别和完整分母、两臂均65帧，不按gate过滤坏帧。25eligible全0通过、40数据不足，pooled和原90%/99%/.1m都拒绝；旧Fast/source/IMU/privdepth35s导航拒绝保留。

## 单次已存输出原因核验

只使用130已存输出和之后的offline标签，不再次推理/训练。去掉bilateral的raw正视差，仅诊断不准入：body overlap原94492/109961=85.93%≤.1m→candidate97106=88.31%，两者仍未到90%。Bilateral丢掉67287→63239个原本正确的raw点，同时保留10→803错误。证据支持有限域适配略改善匹配，但confidence/matching可靠度仍不足；不能删除LR检查救分，raw深度也不是认证自由空间。本次这些body/floor overlap标签超416span数0，所以不能把本次主要失败归因max_disp不足。

同225floor_overlap raw正确15→118，最终0→7；总体floor共同可见量相对floor_all只有225/20433≈1.10%，body共同可见量109961/674893≈16.29%。这些是同序列表面标签比例，不是全3D可观察性或所有视角/运动绝对上限。明白区分匹配失败与相机/遮挡覆盖不足，训练无法创造当前或过去没有观测到的近场。标签仍是当前renderer nearest像素+原pose/ray；旧all-surface exposure/mesh限制保持，不额外认证每个材质、透明面和边界。

## 测试、作者证据和后续准入

6训练几何/右眼符号/mask/occlusion/损失/frozen-module fixture RED6fail.98→GREEN6pass.93s；4完整分母/错保留/null/小样本fixture RED4fail.17→GREEN4pass.13s；345preflight10.55s/349完整Hsuite10.75s。模型/数据/推理实际过程由作者执行，作者最终1091身份核验、300trace/130outputs/schema/全标签计数重算及原字节保持PASS。它是来源拒绝，不是测试通过就等于导航通过。实际图与视频已保存并看预定快照。

![训练和完整验证](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/dense_adapter_fit_and_whole_validation.png)
![实际RGB/离线深度/两模型](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/dense_adapter_actual_rgb_depth.png)
[采集双目视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/dense_adapter_heldout_run/stereo_view.mp4)，[第三视角视频](/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_memory_navigation_20261004_v1/dense_adapter_heldout_run/third_person.mp4)。GT图仅离线显示，非输入。

关闭此唯一decoder适配候选，不再调LR/iters/层/预算或第二fit；不放宽90/99/.1或改相机救分。源测量、estimated-pose、空间记忆安全支持、控制和正式导航仍各自未准入，任务6世界模型前提未满足，整个研究尚未完成。下一项应直接把本次完整RGB深度输出消费为来源可追溯的记忆和扫掠拒绝证据，分开current/history/unknown；这是same-sequence只读失败诊断，而不是对已拒绝模型赋予控制准入或又一个网络。只做一次固定whole序列空间消费，不重推理/采集/矩阵，回答少量准确点是否支持真实可执行路径，而非继续优化all-query分数。若没有合格近场支持，明确需要独立的冷启动/安全起步信息；历史或任务先验必须独立声明，不能写成观测free。formal原35s/纯RGB+估计位姿/唯一世界模型增量仍不假称完成。


## 独立审查与作者范围裁决

ONE fresh readonly whole-stage review C0/I0/M1。个人10fixtures1.00s、1409含重复的SHA比较、520逐帧group计数及pooled p90、旧32输入六空/新26含低样本42/71/39/41眼、130预测schema/65RGB真来源、321同步clock/1601physics/320requests、两PNG均核验。实际GPU训练的670frozen/52changed/158.568s、349-suite和两视频321frame解码继承作者日志与代码断言，未独立重演。

Minor：collection180source没有覆盖直接调用的scripts/collect_data_dubins.py、scripts/bev_ground_truth.py及benchmark/artifacts.py三个helper。主runtime/实际rawbytes与fit/evaluator依赖已封存，但不能宣称采集端所有传递依赖完整预封存。作者接受限制，独立POSTHOC文件保存当前helper身份，不能补成运行前证据，不改变当前已保存数据上的独立拒绝。原审正文按字节归档，不重采/拟合/推理/二审。fresh环境、连续地面、自由体积、估计位姿、实时控制和正式导航全部未获审查准入，作者逐项保留；整体研究未完成。下一项一次记忆/支持诊断不赋予已拒绝来源控制准入。
