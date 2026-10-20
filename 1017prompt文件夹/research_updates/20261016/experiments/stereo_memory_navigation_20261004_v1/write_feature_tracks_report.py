from pathlib import Path
import json,hashlib
H=Path(__file__).resolve().parent;W=H.parent.parent;o=H/'feature_tracks_runs'
p=json.loads((H/'feature_tracks_protocol.json').read_text());a=json.loads((o/'assessment.json').read_text());audit=json.loads((o/'source_audit.json').read_text());sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
lines=['# 因果 RGB 特征轨迹、三角化与稀疏来源记忆：2026-10-04 23:35UTC 阶段',
'', '本阶段已完成唯一冻结 ORB 候选的实际离线推理、当前/历史输入对照、来源记忆重建与拒绝原因核验。**候选来源准入拒绝；尚未形成可靠 RGB 导航，整个研究未完成。**',
'', '## 实际范围和身份',
f"协议封存 {p['sealed_utc']}；全部26对/52组预测保存 {a['predictions_saved_utc']}；标签评估 {a['assessed_utc']}。51项源身份、203项上一阶段产物预封存；最终作者身份核验另见 feature_tracks_verification.json。",
'三组归档：编码板6对（000/060/070/080/090/120）、原E1 16对（000至147）、原hard_0 E1 0004四对（000/010/020/023）。每组开头清空历史和记忆，内部依次推进，只保留前三对过去视图。这是历史开发数据按完整采集序列归组、有限选帧；**不是三个完整10Hz RGB流，不是fresh held-out，不是新仿真或控制A/B**。当前与历史两臂均使用同一冻结结构。',
'固定相机 ball_stereo_front020_up005_v2：X=.20m,Y=±.30m,Z=.05m，640×480，f=584.888889px；标定、原生眼别、旧mask、灯光、PID、planner、动作和35秒规则未改。位姿为归档真实相机位姿，明确属于RGB+known-pose诊断层，未验证RGB+estimated-pose。',
f"实现 feature_tracks.py SHA256 `{sha(H/'feature_tracks.py')}`；protocol `{sha(H/'feature_tracks_protocol.json')}`；预测manifest `{a['manifest_sha256']}`。原项目只读，无提交无关文件。",
'', '## 唯一结构与因果边界',
'ORB1000/scale1.2/8层/edge31/patch31/FAST20/WTA2；按response、y/x/octave排序，每眼最多256特征。mask中心和ceil(size×sqrt(2)/2)旋转方形完整支撑检查；这是本实现的支撑定义，未证明ORB金字塔滤波/插值的每个底层依赖像素都排除机体。不能把它当作完整光学遮挡证书。',
'同/过去眼别通过对称极线≤1.5px、Hamming≤64、forward ratio<.75且至少两个有效竞争者、mutual reverse-best产生对应（reverse不再做ratio）。随后正深度、视差≥.5°、重投影≤1.5px、八个单坐标±1px三角化最大位移≤.10m；同query有效来源重建任意两点距离>.10m就拒绝，否则只取视差最大的来源。**sigma_m是像素扰动敏感性，不是校准概率置信或对应正确保证**，未覆盖联合像素/位姿/曝光误差。',
'当前臂只读同时刻右眼；历史臂另读最近三对严格过去左右眼。predictor包仅xy/desc/size/pose/frame/time_s/eye；不读编码ID、板布局、源网格、renderer深度、真伙伴、未来动作。未来源先检查时间后直接跳过payload。所有特征/预测落盘后才加载离线标签。',
'稀疏记忆保存world、query/source四元组、视差、重投影、敏感性与last_seen；10秒过期、pose_sigma>.05m拒绝读写，快照深拷贝。known-pose测试pose_sigma=0，不表示真实里程计不确定性为零。记录不去重到实体地图，不标floor/free，不认证机器人扫掠、盲区或安全。',
'', '## 冻结完整分母与结果',
'各序列min100左ORB查询，正确/所有查询≥90%，保留且有标签的三维欧氏误差p90≤.10m。未匹配、无独立标签仍留在完整分母。此门槛是来源候选准入，不是像素面积或导航成功率；不倒改V1/Fast/M2或历史35秒结果。',
'', '|归档组|输入|全部查询|保留|≤10cm|完整准确|有标签保留p90(m)|准入|', '|---|---|---:|---:|---:|---:|---:|---|']
for seq,arms in a['sequences'].items():
 for arm,v in arms.items():lines.append(f"|{seq}|{arm}|{v['queries']}|{v['retained']}|{v['correct']}|{100*v['unconditional_fraction']:.3f}%|{v['p90_m']:.6f}|拒绝|")
lines += ['', '板的历史臂343保留/318正确（条件92.71%），但完整20.70%<90%，因此仍拒绝。原E1历史88保留/47正确，15保留没有有效renderer标签，不隐藏于分母；条件53.41%也不能证明可用。hard只有87查询，连min100都不足，唯一保留点错误；不事后增加特征预算或补帧使其过线。',
'历史相对当前增加301个正确query、丢失1个正确query，净增300；不是去重空间点或闭环导航增益。板正确46→318，E1 19→47，hard仍0。这证明某些历史图像提供可用几何信息，同时证明该实现的困难原场景来源质量仍不够。',
'', '## 标签范围与拒绝层次',
'板按有限平面界限离线求射线交点，且当前renderer轴向深度差≤.01m；它是受控几何实现见证。原场景采用四舍五入特征中心处当前renderer轴向深度和原始subpixel射线。边界、无效深度、混合像素以及尚未独立认证的面可能产生标签误差；标签不是全场景独立测量。全查询2637中2502有此受限标签，135无标签均在E1，完整分母保留。',
'原场景只可给出world-Z<.08m的floor标签与elevated_or_other，后者可能包含墙、障碍或其他表面。归档不提供可靠逐像素刚体ID，故**不能完成独立body分区**；mask排除也不等于真值body标签。这是预协议body/floor分区中的数据不足，明确保留；未用此限制删除坏点/放宽gate。',
'', '|组/臂|描述子匹配query|正深度/视差/重投影通过|敏感性通过|来源冲突|最终保留|', '|---|---:|---:|---:|---:|---:|']
for seq in a['sequences']:
 for arm in ['current','causal']:
  rs=[r for r in audit['rows'] if r['sequence']==seq and r['arm']==arm];keys=['descriptor_queries','triangulation_queries','sensitivity_queries','conflicting_queries','accepted_queries'];lines.append('|'+seq+'/'+arm+'|'+'|'.join(str(sum(r[k] for r in rs)) for k in keys)+'|')
lines += ['', '这是原冻结结构的事后只读拒绝链，不是新增筛选器。hard的前三个左查询数为1/0/47，最后39；名义full-support ORB供给十分稀疏，但不能由此断言所有RGB像素永远无信息。历史18个描述子query候选中10通过几何、仅1通过敏感性；几何基线/定位敏感性是有直接计数证据的限制。',
'hard000唯一接受来自左feature0→右feature205：敏感性.016727m、视差14.2108°、重投影.500016px都通过，实际离线三维误差11.003498m。两观测的离线标签位置相距7.692849m，右标签elevated_or_other、左floor：在受限标签口径下有强错误对应证据，单靠重投影/视差不能保证匹配正确。它在2.3秒记忆里仍存在；当前实现不会凭真实深度在线消除它。',
'E1错误对应标签有的非常接近，例如frame060两真中心距离.03729m但输出误差.17870m；不能把所有错误都归因于明显错伙伴，也不能把像素扰动敏感性当终态误差上限。没有在本轮调ratio、换描述子、强行过滤真值坏源或引入另一网络。',
'', '## 记忆、时间与测试',
'终帧板330记录/308正确（其中222旧query）；E1 73记录/33正确/15无标签（54旧query）；hard 1记录/0正确（旧query）。unique_query_provenance仅去重来源身份，不是物理点去重/区域覆盖。10秒过期真实执行，未知始终不转free；拒绝候选后未接入控制消费者。',
'七个手算几何、极线、匹配、未来payload、支撑/记忆测试实际RED7→GREEN7；快照污染回归实际RED1→GREEN，真实predictor两手算点及未来抛错fixture通过；完整既有CPU套件124通过（2.23秒）。没有为只读报表新增镜像测试。作者最终核验重新提取全部26双目feature数组，逐字重建52预测/全部memory，52次追加未来payload抛错源不改输出；验证文件另存。',
'两臂加特征提取的每帧记录CPU耗时.0134—.2344秒；不是实时10Hz性能通过，也不含采集/位姿/地图/控制。无新GPU训练、模拟、视频或导航评测；归档实际采集数据、旧视频和新预测/来源/图均保留。旧board material报告SHA962b39f84008ec487b15bf32b2fb4c669071c3ff945cbff50bbc0165bcd36a47未改，历史拒绝不追改。',
'', '## 实际可视证据',
f"![全分母指标]({H/'figures/feature_tracks_metrics.png'})", f"![原始图像特征覆盖]({H/'figures/feature_tracks_focus.png'})", f"![通过几何却错误的实际对应]({H/'figures/feature_tracks_false_pair.png'})",
f"全部26左眼覆盖图：[{(H/'figures/feature_tracks_all26.png').name}]({H/'figures/feature_tracks_all26.png'})。灰色未保留、绿≤10cm、红>10cm、橙无标签；叠加真值仅离线展示。",
'', '## 分层结论和下一项有界工作',
'接口/因果重建在这些正常有限时间输入下通过；真实估计位姿未验证。物理深度来源质量：唯一候选拒绝；稀疏记忆数据结构及过期/拷贝语义通过，不等于可靠环境记忆；没有planner消费、控制A/B或新导航成功率。所有错误/缺失按原分母交付，不用高对比板替代原场景结论。',
'结束本轮ORB结构改进分支，不继续换网络或扫描ratio/阈值。下一项仅一个**原场景固定主动观察动作的fresh完整采集见证**：沿可追溯实测短段使用冻结roll PID/请求，不改变相机、正式障碍材质或unknown规则；开头重置、内部不重置，预封存最大18秒、一条完整请求及逐0.1秒双RGB/曝光位姿、原来源gate和碰撞/覆盖/进度指标。先核验其初态和真实扫掠适用性，若没有可证明适用的受控动作就不启动并给出具体阻塞；不能以unknown地图宣称安全。',
'目的隔离当前少量选帧/固定前视下的有效观测供给与实际主动运动能否增加真实可追溯来源；仍用冻结结构作回归见证，旧候选拒绝保持。不是新描述子候选或导航成功；不得为补min100事后选帧，不允许失败后继续其他动作扫描。随后只有来源/局部支持实际通过才消费，估计位姿、唯一学习增量、原35秒E0/E1/E2仍待分层完成。全任务未完成、automation保持ACTIVE。',
'', '最终一次独立只读全阶段审查与作者最终核验保存在 feature_tracks_review.json / feature_tracks_verification.json；审查范围和延期小项若有，另附最终审查记录。']
(W/'reports/benchmark_causal_feature_tracks_and_sparse_memory_20261004.md').write_text('\n'.join(lines)+'\n')
