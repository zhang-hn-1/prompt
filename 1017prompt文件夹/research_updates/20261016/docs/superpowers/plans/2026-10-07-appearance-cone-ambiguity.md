# 合格外观范围 → 多视角几何歧义集合（一次独立候选）

前置外观分类在10个已知位姿development曝光全通过95%precision/90%coverage；不能转译成深度/占用。已经看过这些曝光GT，今后只能称历史development，不称fresh。保留其他所有拒绝。接续唯一低阶候选：当前15s已知的10个外观mask与过去camera poses构造静态object视觉锥相容体积。该体积是potential occupied hypothesis，不是观测或已证实自由空间；不接入控制。

固定ROI由当前root与已知pointgoalXY包围盒各扩展2m、Z=.1..1.5，.1m格心；不是GT障碍边界。USD optical变换按既有实际约定。每个voxel投影到native像素，完整7x7范围在FOV且无当前CAD才算有效约束；FOV外/CAD遮挡是未知而非反例。至少两个不同曝光可见，有效视图中>=90%落入外观mask，保留为几何歧义集合。ROI外仍未知。不声称凝聚体积所有点均实体，更不以排除体积证自由空间。源150/每个旧RGB clock/pose/mask clock可追溯，假设统一出生15s，不刷新原双目geometry出生，不读取未来。

生成并seal所有voxels/每视图支持统计/旧input SHA后独立验证：对应10个whole历史曝光全native当前真实hazard .5..3/外root>.55域，真实表面到保留voxel距离<=sqrt(3)*.1/2的覆盖>=95%；且从可见体积投影出的最近表面相对独立GT的precision>=95%（缺失GT计分母）及p90误差<=.10m。同一geometry .10门槛不放宽。至少100有效预测点每眼才可exercise。ROI截断必须报告，不能忽略miss。只是development前置，PASS后仍需fresh whole sequence与online接口；REJECT即不换ROI/pitch/支持阈值/形状先验救分，不调用新网络。所有正式与动作A/B尚未触发。
