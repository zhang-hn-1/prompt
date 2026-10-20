# 球型机器人：请求响应核验、空间双目候选与位姿估计

本轮接续未完成的请求消费审计，随后直接完成两项实际CPU实验，没有停在审计结束。没有新物理采集、标定矩阵、planner搜索、网络拟合或控制改动。研究整体未完成，旧35秒失败、RGB深度、M2与停止包络拒绝保留；不报告新导航成功率。

## 1. 请求与响应：核验完成，整表替换不准入

形式化planner42种primitive：正反向×主轴1.5/1/.5×副轴0/±10/±20/±30°，曲线取V1 curvature(main,aux)。实际跟踪器以几何pure-pursuit产生曲率，用**当前实测XY速度**查询BankMap逆映射，编码bank/.75并叠加外环反馈，再经协议±30°、roll-PID、smoother和supervisor执行。当前默认profile曲率仅作诊断，不能把历史blend当作当前逻辑。

前轮唯一新稳态点支持main+.5/rawaux−30°：真实XY曲率.792373478，BankMap原点.796603322，V1 .880051966。两误差都在原.238980997容差内；不能宣布V1原gate失败、把优势归因于PID增益或移植整张表。假定稳态圆弧的.3m primitive，V1与实测曲率所给终点只差3.93858mm，航向约1.51°；这只是名义几何描述，不解释完整导航偏离/瞬态。原common-domain cap和time-cost候选拒绝不被救回。

来源/精确旧实测数值：response_consumer_audit_20261006.json、bank_response_assessment.json。原部分接续receipt保留不覆盖。新审计没有policy二次act，没有未来实测速度/反馈作为输入。

## 2. 唯一空间代价候选：实际运行，拒绝

沿用旧hard0004四帧000/010/020/023的原RGB、mask、K、当前和严格过去已知相机pose、128个.5..3m逆深度格和5x5 ZNCC。独立clone只提取完全相同的原始代价；八方向路径加入P1=.05/P2=.20固定邻域惩罚，不扫描参数。

路径递推依据[Hirschmüller原论文](https://elib.dlr.de/55367/1/Stereo_Processing-Hirschm%C3%BCller.pdf)公式13/14。本实现改用ZNCC与逆轴向深度标签，未复制论文的互信息代价；不是重跑先前OpenCV SGBM矩阵。

空间选择是**推断假设**。要标为evidence-qualified，还必须满足原unary有限/<=.2/距原最优<=.05、原完整competitive区间半格扩展后ray宽<=.10m、winner在内点。无效像素仍未知；缺失处重启路径而不补洞；不生成free map。邻域惩罚不是新的测量。

四次预测原六数组与历史independent-unary baseline全部逐数组精确相同；实际预测全部保存后才载入GT标签。所有来源时间/眼别/位图及原unary完整volume均保存。此为历史开发数据+known-pose诊断，不是fresh、实时或RGB估计位姿导航。

| 帧 | 全分母 | 旧正确 | 新合格输出 | 新正确 | 新finite p90 m | CPU s |
|---|---:|---:|---:|---:|---:|---:|
| 0 | 29792 | 19 | 302 | 18 | 0.886623 | 17.067 |
| 10 | 17273 | 12 | 217 | 10 | 1.395807 | 26.310 |
| 20 | 12742 | 8 | 108 | 8 | 1.171123 | 33.530 |
| 23 | 13576 | 4 | 142 | 4 | 1.105319 | 44.491 |

总73383查询，旧43正确→新40正确；新769个合格输出，完整正确率0.054509%，条件正确率5.2016%，finite p90=1.156352m。17543个查询winner改变；新增正确0、丢正确3、移除错误27、新增错误3。不是行为等价，但没有形成可靠深度。

四帧全部未过原每帧>=100、全体正确>=90%、finite p90<=.10m门槛。拒绝此候选，不继续调P1/P2/texture/depthrange/source集救分。即使空间邻域参与，错误光度极小值仍主导；这只否定本固定结构，不认证所有空间方法必然无效。raw假设30920有限、raw p901.147696m，和合格输出分开。严格全2x2来源可见且正确的标签只有1个，不能把其余输出称作可靠观察。没有地面扫掠/盲区free或导航准入。

## 3. 独立双目VO：整条18秒序列实际执行，拒绝

直接继续原场景active E1的181帧历史序列。复用从原RGB+mask独立提取的ORB xy/desc/size，另存181份去掉全部world-pose字段的payload。算法只读前一帧左右/当前左特征、固定K和一次静态左右光学相对标定；frame000同拍pose相消得到基线.600000024m，作为明确的已知安装标定，不是GT轨迹输入。

立体点复用旧epiline1.5px/Hamming64/ratio.75/mutual、parallax.5°/reprojection1.5px/敏感度.1m。时间描述子对应不用真pose epiline。唯一EPNP RANSAC100/1.5px/.999/seed3072后ITERATIVE，至少8点、8inlier及比例.5、inlier p90<=1.5px；当前←前一相机变换求逆累积。失败永久置未知，不identity填补、不reset、不读IMU/encoder/root quaternion。

所有181预测先落盘，之后用GT相机世界pose核对；初始pose仅用于评价坐标规准对齐。预先冻结新pose诊断门槛：>=6s的121对完整分母中>=90%平移<=.05m且旋转<=2°，finite p90同门槛；全序列有效累积覆盖>=90%，位置RMS<=.05/peak<=.10m，旋转p90<=2°。不改旧depth/event/nav gate。

180/180对全部因合格双目点不足8而拒绝，在进入PnP前失败；没有运动误差finite样本。累积仅frame000恒等规准有效（1/181=.552486%），其零RMS/peak是初始点的平凡结果，**不代表估计精度通过**。>=6s完整121对，无finite估计，完整正确0%；拒绝。缺口来自此固定rig/原序列下可用双目三角点支撑；不能归因于PnP调参，不降低8点门槛救分。没有fresh泛化、视觉里程计成功或导航成绩。

## 4. 运行、测试与身份

空间协议预封存2026-10-05T17:09:09.626758+00:00；四帧实际2026-10-05T17:09:09.832623+00:00→2026-10-05T17:11:21.365448+00:00，runner exit0；独立audit exit0。VO协议预封存2026-10-05T17:17:49.164652+00:00，181帧序列exit0，audit exit0。无新视频，因为没有新物理运行；active_observation_run两部真实18s视频与endpoint_navigation_run原35s视频仅作历史上下文。

空间callable RED9fail3pass.21s→GREEN15pass.35s。VO RED4fail1pass.17s→GREEN5pass.16s，真实时间guard RED1fail6pass.22s→GREEN7pass.20s。合计最终全目录268pass9.51s，含旧246回归。最初全suite没带USD library导致libtf.so import失败；第二次环境正确但包含当时尚未实现的VO RED4fail262pass。均保存，不删测试、不当科学失败/成功；最终是完整正确环境+实现后全suite。

原始数据、独立源码/scaffold/所有失败日志、协议、GT隔离标签、完整unary及预测、来源与版本hash保留。源码/配置身份代替不存在的Git BASE；原项目只读，本目录是独立实验边界而非已注册Git worktree。没有无关commit/upload/delete。

图：spatial_vo_measurements.png。代码/数据：spatial_cost_protocol.json、spatial_cost_predictions/、spatial_cost_audit.json、stereo_vo_protocol.json、stereo_vo_inputs/、stereo_vo_predictions/、stereo_vo_audit.json。完整研究NOTcomplete，formal E0/E1/E2仍未触发。后续实际推进不得把一次steady点、只读interface或拒绝模型写成导航通过。

作者核验与第二人审查将在独立附录中署名，前述正文不倒改旧报告。


## 独立审查与作者追加（保留原正文）

ONE fresh gpt-6-astra/high只读审查：0Critical/0Important/2Minor，完整原始审查见spatial_vo_review_transcript.md，作者逐项范围裁决与风险见spatial_vo_review.json。评审个人22fixture通过.40s、185预测+4unary身份、原六数组复算精确、181去pose特征与原RGB来源、全部protocol identity、分母/误差/静态标定/来源时序核验；未重跑原推理、全268suite或物理/视频。作者核验spatial_vo_author_verify.json另署，不冒称第二次审查。

两Minor仅追加澄清、不修改原图/已审正文：第一幅图的柱为**正确数量**，19/18、12/10、8/8、4/4；对应完整分母29792/17273/12742/13576，y轴斜杠不是归一化比例。表CPU s/图CPU seconds是**CPU实现的elapsed wall-clock latency**，不是process CPU time。evidence-qualified只是固定原unary筛选条件，不等于真值认证；strict完整来源正确只有1点。旧gate/候选拒绝不改。

审查后没有因小阶段完成停止：已另开启理想gyro+球壳encoder滚动里程计阶段（独立spec/plan/protocol，未包含于这次review边界）。5fixture RED4fail1pass.20→GREEN5pass.13s；两条历史18/32s完整2502样本递归预测保存后才离线评分，t>=1全分母850/1550。RMS.046126/.047353m、peak.046278/.049652m、attitudep90 1.270/1.605°，按预冻结理想传感器诊断门槛两条通过。不是fresh/real IMU/online验证、不是纯RGB、没有模型或导航准入；完整新阶段仍需在线shadow与单独收尾。初始化落地导致的持久误差、零噪声/无bias及历史DOF读回来源限制保留。不得把这次有限pose通过写成旧RGB深度恢复。
