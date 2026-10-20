from pathlib import Path
import json
H=Path(__file__).resolve().parent;W=H.parent.parent;o=H/'moving_coded_board_run'
s=json.loads((o/'summary.json').read_text());a=json.loads((o/'geometry_assessment.json').read_text());p=json.loads((H/'moving_coded_board_protocol.json').read_text());status=json.loads((H/'moving_coded_board_status.json').read_text());m=json.loads((o/'missing_photometry_labels.json').read_text())
rows='\n'.join(f"| {r['frame']:03d} | {r['eye']} | {r['expected_corners']} | {r['recovered_corners']} | {r['current_rms_px']:.3f} | {r['current_p90_px']:.3f} | {r['past_p90_px']:.3f} | 未通过覆盖 |" for r in a['rows'])
missing='\n'.join(f"| {r['frame']:03d}/{r['eye']}/{r['tag']} | {r['white_border_cells']} | {r['best_hamming']} | {r['margin']} | {', '.join(r['rejected_by'])} |" for r in m['rows'])
text=f'''# 球型机器人：移动编码板的图像与位姿核验

本轮实际跑完一条12秒前进—停止序列，并保存双目视频、图像、物理与执行数据。新增板上的图案可以由RGB独立识别；先保存识别结果，再拿板的真实位置核对。

已识别的356个合格角点全部位于真实投影2像素以内，各眼各时刻RMS .620—.809px、p90 .794—1.065px。在7、8、9秒运动时，前一个.1秒位姿的p90约2.02—3.29px，当前位姿约.79—.96px。这支持当前采集事务对**这些真实RGB特征**的对齐，不能扩展到所有表面、运动或曝光。

但预先冻结的完整gate仍未通过：420个几何合格角点，只独立识别到356个，覆盖84.76%，低于90%。不能删掉漏检位置，把条件下的小误差变成总体准入。因此按照事前规定，**没有执行后续V1匹配，没有新RGB导航或成功率**。旧V1、Fast、历史采集与导航拒绝结论不改。

## 唯一实际实验与身份

协议V1预封存21:41:33UTC；沙箱第一次启动未能访问GPU，21:41:52—21:41:58退出-11，只产生请求文件与空frames目录，没有板、图像或物理序列。日志保留于moving_coded_board_startup_failure.log，失败status与目录原样归档。获允许的GPU环境后执行唯一实际采集，未增加物理实验预算。

采集前CPU前视图fixture发现板作者坐标的镜像问题：名义朝前位姿下，世界+Z向图像右、世界+Y向图像上；光学+Z为前方，编码列应沿-worldY、行沿-worldZ。只修板作者的单元方向及离线角点对应，不增加镜像搜索、不用真值帮decoder找角点。原V1协议/作者/测量源码字节快照保留，新增fixture RED1→GREEN7。V2在{p['sealed_utc']}重新封存18项源身份；原36码、布局/尺寸、decoder、所有数值gate、相机/灯光/请求不变。此时尚无任何实际图像或物理数据，不是看过结果后改口径。

实际运行{status['start_utc']}—{status['end_utc']}，退出0：120请求块、121capture、601物理采样；两个视频各121帧/10fps，没有旧runner计分外尾帧。6秒零请求→3秒主轴+.3rad/s、副轴0→3秒零请求，只在开始reset，内部不reset物理/PID/smoother。由121个10Hz实测capture位置构成的XY全程折线路径{s['root_xy_path_m']:.5f}m、净位移({s['root_xy_net'][0]:.5f},{s['root_xy_net'][1]:.5f})m，6秒后净位移({s['post6_xy_net'][0]:.5f},{s['post6_xy_net'][1]:.5f})m，同121个10Hz capture速度采样的XY速度峰值{s['actual_xy_peak_speed_mps']:.5f}m/s。601个50Hz物理采样的路径为.552596886m；采样频率不同，二者不冒充同一折线统计。

与历史motion_render_ablation_probe同完整12秒forward序列，17个非raw物理字段逐值相同，包含raw请求/protocol/smoothed/applied、关节、实际位姿/速度与相对时钟。本轮减历史raw初始化原点为-.02s/-1物理步，单独披露；不是19字段全部相同。板无Collision/RigidBodyAPI，只改变诊断图像，不改变实测运动。旧来源保持只读。

相机ball_stereo_front020_up005_v2固定X=.20/Y=±.30/Z=.05m、640×480、f=584.8889px与.60m基线；光源fixed_camera_fill_v1、mask、PID/限速/监督器和35秒正式规则不变。冻结事务为fixed3→单次basecapture→fixed3→替换图像/视频；每次不额外推进物理、重复policy或累积history。导航marker仍隐藏。已知板是独立开放平面的标定干预，不替换正式障碍材料，不把板或ID作为导航目标。

## RGB测量与离线标签分开

6个预定capture索引000/060/070/080/090/120×双眼。RGB检测只读实际图像、自遮挡mask和预封存二进制码表；当前或过去位姿、board世界角点、renderer depth不进入detector。一次全图轮廓/四边形/固定bit解码，无真值位置指导搜索。全marker像素mask支持；不确定、裁剪、光度破坏或解码竞争保持缺失。36码5×5内容、7×7含黑边、最小跨旋转Hamming6；侧长.28m/格.04m，6×6间距.34m，物理X=初始相机中点X+2m，Y依初始中点，Z=.20+.34row。无subdivision、无碰撞；白背景在板后.002m，避免共面相争。

全部RGB检测保存{a['detections_saved_utc']}，才载入独立板几何及实测当前/过去相机位姿；标签评价完成{a['labels_evaluated_utc']}。世界角点按保存USD列变换→光学diag(1,-1,-1)投影，不拟合位姿、内参、时间偏移或挑最好lag。过去对照只用严格前一个实测.1秒capture；000没有过去，表中与当前相同，不算动态同步对照。

几何分母是完整tag投影在画内且不触固定mask、边长≥20px/面积≥400px的全部四角，没按识别成功/GT误差挑点。门槛每个眼/预定帧至少12期待角点，≥90%找到且≤2px，RMS≤1.5px、p90≤2px。期待tag仅视锥/mask标签，不能认证每条3D射线的所有遮挡。下面表保留所有12组，缺失是缺失，不等于已证明错曝光。

| 帧 | 眼 | 期待角点 | 实际找到 | 当前RMS px | 当前p90 px | 前一capture p90 px | 原gate |
|---|---|---:|---:|---:|---:|---:|---|
{rows}

同一时刻左右完整可解码ID交集在六帧均为空。相机实际看到不同的板区域；**完整tag交集0不等于所有像素双目交集0**，不能推成普遍双目不可用结论。它提示需要部分特征/时间记忆，而不是要求每一整块marker同时看全。

## 为什么漏检：保持拒绝，拆出观测失败层

16个几何期待、RGB未识别的tag全部保留。检测落盘后做一次明确**GT辅助的离线真四边形warp**，只是核验漏检原因，不是另一decoder、不写回检测点、不提高覆盖或进入模型输入。

即使给真四边形，16块仍全部被同一decoder拒绝。其黑边24个采样格被灰度阈值分为白色5—13格，超过原最多2格，部分内部码也损坏（最优Hamming0—7、margin0—6）。全部块都有足够全块灰度变化；不能解释为“仅轮廓proposal没找出来”，也不能删mask/放宽Hamming或黑边门槛救准入。下面公开全部漏检，没有只挑一个图。

| 帧/眼/ID | 黑边误分白格 | 最优Hamming | margin | 拒绝原因 |
|---|---:|---:|---:|---|
{missing}

原生RGB中可见一侧黑白图案被亮斑洗淡，对应已知黑边分类破坏；当前pose下其余可靠角点精确定位，较支持“可用特征和局部光度失真并存”。但尚未隔离材质反射、光照/色调处理与真实遮挡的各自贡献，不能单靠亮斑就认证specular是唯一原因。已知几何warp仍不提供独立漏检角点位置，也不能将其误差按0计入原gate。

## 门槛设计缺陷与没有做的下游

本轮匹配预算max6帧×12锚点=72，却沿用min100 pooled gate，数值上无法达到。该缺陷保留在原protocol，没有看完结果后加锚点/降低门槛；它不算V1的性能失败。实际匹配在更早的几何覆盖gate已阻断，0次V1执行，两个问题分别报告。以后独立新协议必须在采集/拟合前对预算与min样本检查，不追改本轮。

7项新增有意义测试（前6缺实现RED6→GREEN6，采集前作者前面朝向RED1→GREEN7）；完整实验区111项CPU测试通过。它们覆盖旋转ID与角点对应、白边伪候选、全块mask拒绝、YZ板与USD列/光学坐标、真实前面投影能不借GT解码。没有新增网络/训练、没有控制A/B、E0/E1/E2或新导航成功率。

## 图与实际视频

红线=独立RGB检测角点，绿线=离线真实板投影；绿框没有红框的全部位置继续计缺失。近静态000与运动070的原生图：

![静态与运动双眼]({H/'figures/moving_coded_board_static_motion.png'})

![全部预定帧]({H/'figures/moving_coded_board_all_frames.png'})

下面是真值四边形截取的**离线漏检标签图**，不是模型有效观察或新的识别结果：

![全部16个漏检标签]({H/'figures/moving_coded_board_missing_labels.png'})

实际视频：[双目]({o/'stereo_view.mp4'})、[第三视角]({o/'third_person.mp4'})。数据、请求、physics、history、时钟、board身份、RGB检测、逐点误差、测试/启动失败日志均保留在{H}。

## 独立只读审查与最终身份范围

唯一独立只读审查没有Critical或Important问题，独立运行111项测试通过，并重建12组RGB检测、全部420/356分母与误差、16块漏检、两个完整121帧视频、17非raw物理一致性及18封存/26旧产物身份。审查正文SHA为`08782e16a995c14347b12f92e57ca38852de4180f895773e4205e93d7226df83`。

两个Minor仅涉及报告表达：将名义前视图的世界轴与光学轴分开，并给路径/速度统计标明10Hz capture采样。作者核对fixture和统计实现后修正文字，原始数组、summary、协议/门槛与拒绝结论不改；本段与这两处文字在独立审查后加入，不冒充新一次独立审查。完整记录保存于moving_coded_board_review.json。最终验证器由作者执行，审查者只检查部分断言/输出，没有全面审查或执行它。

## 下一项有界实际工作

不重跑本轮图案/网络/阈值矩阵，不继续延时或PID扫描。基于黑边在真实RGB被亮斑破坏的具体证据，下一项最多**一次板材质单因素A/B**：同初态/同12秒请求/同码/同板位置/同相机与灯光，复用本轮原默认材质B0；唯一改动是明确低反射、高粗糙度的板自身材质。不能同时改曝光/照明/相机/图案或解码门槛。这是对局部光度失真的因果检验，不替换正式场景材料。

新协议在任何新数据前独立封存完整几何gate及对应样本预算；旧gate/失败不可回改。比较完整期待角点的漏检、黑边光度破坏与当前/过去误差。若材质单改未解决，就明确拒绝该解释，不再换灯/颜色/阈值直到通过。几何通过后，才在同冻结V1与一个合理预封存预算上检验来源质量；无未来/板真值喂模型，不认证原无纹理障碍或视觉导航。随后才决定一个有证据的因果RGB时间记忆表示，而不再把全研究变成无限采集诊断。

整体研究未完成：真实RGB可靠来源、估计位姿、在线来源记忆+可执行局部规划、唯一学习/世界模型增量与原35秒E0/E1/E2仍需分层验证。automation继续ACTIVE；没有请求用户修改硬件或扩大任务。
'''
(W/'reports/benchmark_moving_coded_board_rgb_pose_20261004.md').write_text(text)
print('REPORT_WRITTEN',len(text))
