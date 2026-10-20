# 独立来源候选：冻结双目metric一致性核验

前一全历史MODEL池因一个跨时颜色虚假核心阻塞planner，已独立REJECT，不改窗口/筛选重新计分。本项处理已实际定位的来源几何歧义，唯一新增环节为原raw颜色核心是否有当前/过去同窗口metric双目支持；其他相机/照明/NN/分类/四曝光/位姿/.2 inflation/记忆/PID/规划margin全部不变，不扫描阈值或替换网络。

来源规则冻结：原source_packet是实际冻结NN confidence、L/R协议和动态CAD过滤后的点，须当前source时刻匹配；其query/partner pixel的原RGB分类均正，worldZ>.05且<=1.5。把这些完整同曝光双目点缓存，原颜色核心只有距离该原四曝光窗口至少一个metric支持点3D<=.20m才可进入独立核心池。该.20是预先冻结的结构一致性容差，不能冒充统计NN置信界/原来源可靠性已准入。原raw metric source旧拒绝保持。保留被接受核心对应的actual point UID/frame/时刻/距离，颜色窗口/推导时刻另记录；没有metric支持不以先验补齐，不用未来、GT、episodeID或真实footprint选点。

优先历史14秒全部窗口作开发预检查；它不是fresh。先封存actor输出/每点支持/provenance，再读GT。来源条件仍覆盖>=.95、面积<=6、贡献分类>=.95；额外检验所有累计核心距真实3D几何独立诊断，不反向筛选。原planner start .8707107和local kino均要通过才触发实际；不将只有删假点称导航进步。若缺覆盖或阻塞则明确独立候选拒绝，不继续metric tolerance/窗口/模型扫描。TDD→历史原pipeline重建→prediction封存→独立score→first scoped review/全套→实际同14秒前缀单消费者B0/C1（仅通过后）。完整60秒条件任务与原35秒分开，旧pool不能变成已通过。
