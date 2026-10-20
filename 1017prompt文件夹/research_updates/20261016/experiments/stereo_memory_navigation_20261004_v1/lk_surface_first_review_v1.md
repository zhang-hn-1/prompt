# LK来源与分位数修复独立审查

reviewer `/root/first_primitive_direction_first_review`：Critical0，Important0。独立4项LK+3项分位数测试通过，0.61秒；核验封存/依赖哈希。未实际仿真、推理或真值评价，未改文件。

当前/过去LEFT RGB、CAD、已知相机GT位姿仅诊断；未来payload仅读时间后跳过。LK方向/原生像素/旧几何过滤一致。v2保留无限误差质量，p90=0.0370026203米；v1原错误评价字节保留。59点57在10cm内，总量不足100、目标0，旧目标覆盖要求不通过，REJECT保持。

Minor：旧seal的no_GT_or_future_inputs过宽；独立seal_semantics_clarification_v3明确“无GT深度/实例与未来输入”，已知GT相机位姿依然仅读入诊断，不是RGB+估计位姿准入。
