# 唯一强双眼历史核心记忆候选

状态：预检查设计冻结；整个研究未完成。此前14秒摇摆末端gate拒绝及原D4来源拒绝保留。

问题：摇摆期间原source曾取得完整可能障碍支持，但四曝光窗口及immutable初生记忆不能保留。只改变MODEL核心记忆消费，原双眼/两曝光筛选、RGB分类、.20m inflation、位姿、相机、照明、PID、摇摆日程及planner均冻结。不得调用单眼候选、真实footprint选点或选最有利时刻。

候选：累计固定0–14秒全部29个2Hz源窗口的原raw_world_voxel_centres，按世界坐标去重；仅在最终输出对原核心统一inflate一次。每个核心保留初次/末次推导窗口时刻及来源frame，窗口的完整support frames/times另存，绝不称其为逐点测量时刻。分类模型仍在原2秒出生，不重新拟合。MODEL静态持续是声明的先验，measured/free均保持false。14秒池冻结，之后不刷新、不纳入移动时新可能几何。无核心则拒绝，无fallback。

因果合同：读payload前验证当前源时刻及frame、单调严格顺序；拒绝未来支持窗口、非hypothesis输入、改动radius/class birth。输出拷贝隔离；不把新池计算时刻14秒写成旧分类模型出生2秒。pending包只能空、不能发明核心。所有历史窗口入池，不根据标签质量筛选。

先TDD，再历史预检查：使用已关闭bank_rock_observation_run_v1。旧数据已用于设计，不fresh。核验封存SHA；先累计/封存候选及其源窗口，再读GT标签。所有非空贡献窗口的所有class masks均需precision>=.95，footprint覆盖>=.95、possible area<=6m²。单独用冻结TrackingTubeGrid和当前14秒估计位姿核验start raster .8707106781186548及local kinodynamic可执行，不调用policy.act。新池hypothesis形成时刻和源分类模型出生字段应通过独立桥接明确区分。

任何前置失败即拒绝唯一记忆结构，不继续换窗口、radius、阈值、网络或摇摆幅度。历史预检查不能冒充在线接口或导航。若通过，先首次局部review与完整CPU套件/身份冻结，再两条独立whole-sequence实际B0/C1：相同14秒前缀，B0固定最后当前强集，C1固定全历史核心池；首次planner.act在14秒后，序列内部不reset。60秒总预算、arrival .2m、无contact/collision、原pose门槛、来源95/6、真实进度/障碍通过/guard及视频分别报告。核验前缀物理/请求/执行/源身份，非一致则不称严格A/B。仅记忆消费者不同，固定未知探索先验单独标注，不冒称原35秒严格导航。
