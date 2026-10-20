# 双目metric正支持：改善局部规划，但完整来源拒绝

独立来源候选REJECT_UNIQUE_METRIC_SUPPORT_SOURCE。保持原NN/分类/四曝光/.2/相机/光照/位姿/PID/planner，在历史原颜色核心上只要求其原窗口中至少一个同曝光左右class正的实际metric端点3D距离<=.2。全部14秒源按时加入，25窗口14648 raw occurrences→1724 retained occurrences→123 unique core/.67m²。class最小.992937通过，冻结raster start1.2m、route与local kino通过；但覆盖只有134/185=.724324，故未启动实际导航、不扫描容差补分。网络旧拒绝不因此改成可靠。

实际TDD 3RED缺模块→2PASS/1fixture误用empty floatUID失败→修正为真实integerUID→3GREEN。623全CPU39.97秒通过，预检查84855退出0。第一次限定review C0/I1/M1：本人3tests及actor-only重建25窗口，逐证明都在原support_frames内/过去/<=.20，没有读GT；仅检查结果不能当本人实际GPU。Important要求独立3D几何诊断，作者追加封存后audit v2退出0（v1括号syntax错误日志保存），不筛选actor或改gate；Minor malformed width640.9 int截断可接受，延期，实际冻结数据width全640，未来复用应修合同，不再审查循环。

封存后真值诊断：123核心真实占据volume距离max.248850/RMS.059568/p90.126109，80在真实volume内；surface距离max.282609/RMS.117721。volume与surface不同，不声称NN<.2误差证书。51未覆盖footprint格X[-.10,.35]、Y[-.35,.15]，不能仅凭“背面”说全部不可见。没有把GT用作保留点或改模型。

直接接续另一项有界因果可行性审计（不是新候选准入）：837旧pool核心与全部0–14秒actualmetric source射线，对.20固定axial差及2pixel采样导出的sqrt(2)pixel邻域封存反证UID后再读标签。34核心被更远端点反证、205真占据核心中0被反证；保留MODEL覆盖185/185、面积1.95m²。这说明正匹配会丢未测背面，而因果反证在这条历史开发序列上尚未显示删除真实占据。不能称source/online/导航通过；没有写measured-free，也没有启用actor。下一项若实施，须独立冻结负反证消费者，保持之前两个拒绝，first review、CPU、fresh整序列实际验证后才能报告导航。不能凭此次开发零误删称全域或NN可靠。

文件：metric_support_source_preflight_v1（actor先封存/实际metric pointUID证明/score）；metric_source_3d_geometry_audit_v2.json；free_ray_contradiction_audit_v1（actor式反证先seal后GT）；实际旧14秒视频保留。原35秒/学习增量/整体研究尚未完成，无GPU活动。
