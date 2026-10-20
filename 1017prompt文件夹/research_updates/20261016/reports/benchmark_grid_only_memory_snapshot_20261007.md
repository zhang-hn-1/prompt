# 空间记忆计算瓶颈：唯一 grid-only 精简与严格等价验证

没有改变本次正在运行的导航。旧 PointSupportMap.snapshot 每帧深拷贝全部 raw记录，而 PriorRGBGrid 只消费网格，不使用返回records。独立只读 grid_arrays_snapshot 直接计算同一网格、省去未消费的record复制；raw source/mapper.records和查询身份均保留。它没有缩短TTL、删观测、改未知/占据/floor语义或把prior变成观察。

唯一候选在完整历史226帧上通过：1356个保存网格/先验数组、2712诊断值全部精确一致；最大77155条raw记录。四个事前固定hotstate中snapshot速度比12.25/25.19/20.34/15.03，median17.686，满足预冻结≥2×。**只允许用于posthoc verification优化；没有在线动作改变或新的导航成功。** 不等于整体导航十八倍，更不是实时通过。

## 冻结方法与接口

spec/plan `2026-10-07-grid-only-snapshot*`。保持旧causal update、query原始时间、10s TTL、sigma/pose_sigma、rounding/地图bounds、floor/hazard条件和冲突hazard优先。numpy indexed max只替代逐record算grid的CPU流程；输出grid fields与旧同名fields逐项相等，明确不返回records字段。旧snapshot代码完全保留；调用者不能把新API当成原完整record接口。

`grid_only_snapshot.py`、4literalfixtures含floor/hazard冲突、history/current/TTL边界、outbounds/不确定/ignored、badtime-before-payload、原record无突变。RED模块不存在→GREEN4pass.15秒。有限测试覆盖普通Python float payload；未证明任意自定义/异常数值record类型可等价，mapper causal update约束保持。

历史 `masked_memory_history_run` 226完整act maps是旧生产者原保存值，0/60/145/225固定timingstates；没有随机拆frame/拟合/新GPU。每帧从封存source包用原update生成records，新gridfields对照原实际map六数组与12diagnostic，包括root-current footprint、安全起步区、task-ground prior。raw record state在四timer时前后深拷贝核验不变。所有记录最高77155，不删证据节约时间。

| 帧/records | 旧完整snapshot s | 新grid-only s | ratio |
|---|---:|---:|---:|
|0/9327|.104289|.008514|12.25|
|60/48894|.609219|.024189|25.19|
|145/47961|.585590|.028794|20.34|
|225/10536|.117403|.007809|15.03|

每state一次顺序wall测量、同host背景GPU运行，没有隔离微基准、多次统计或端到端帧率认证。旧函数还返回被消费者丢弃的record副本，新故意不返回，比较就是消除该未消费工作，不是数学算法精度提高。

## 真实失败与后续修正

第一次auditor退出1：NPZ保存frame/time的0D ndarray直接送给旧records，产生unhashable query identity。在第一帧提交之前失败，没有地图/科学结果。原v1脚本、protocol、log全部保存。独立v2仅restore_packet_scalars将frame/time恢复原生成接口的Python int/float，不移动world/改sample/threshold。新回归importRED→GREEN1，连4grid共5pass.24秒；`npz_packet_scalars.py`输入dict不修改、world数组不复制/改值。这是载入行政类型修正，不是新候选。

`audit_grid_only_snapshot_v2.py`实际完整执行exit0，283源/输入SHA、1356数组2712diagnostic精确、四timer原records不变；协议与结果 `grid_only_snapshot_protocol_v2.json` / `grid_only_snapshot_assessment_v2.json` 为权威，v1不覆盖。

独立posthoc `VerificationGrid` 机械复制原update_points，只替换一个snapshot call；逆派生方法精确，旧在线producer/source/PID全不动。current/history两个完整小fixture含future guard，真实importRED→GREEN，连上述5共7pass1.25秒。此adapter只进入后续fullbaseline posthoc auditor，没有进入已经真实运行的35s轨迹。因此fullbaseline里的actual结果不受优化影响。完整H397suite20.75秒通过。

后续独立fullbaseline source71/currentmap350/same-live-historyshadow350重建也实测精确2100地图数组/4200诊断，作为不同完整源序列的消费回归；没有将它称学习模型fresh validation。本阶段源mapper正确性不代表source几何正确，原depth/pose/nav gate全部保留。唯一candidate math/consumer性能PASS，whole research未完成。ONE独立终审另附，raw/source identities和全部失败日志保留，不删除原始记录。


Joint single fresh final review: C=0/I=0/M=1. Reviewer personally checked seven fixtures, inverse derivations, bounded hashes and full saved request/clock/prefix denominators; full source/map census, 397-suite, videos and timings inherited from author. Minor: rounding fixture lacks literal half-cell/negative-boundary cases; no claim that these boundaries were separately exercised. No runtime error identified; unchanged np.rint preserved. No second review or scientific rerun.
