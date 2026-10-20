# 单一局部规划 primitive 来源候选

72秒唯一必要当前序列三固定tail通过必要稳态一致性。实际正bank到达2.5秒、转向反转4.8秒，显式保留有限响应/未认证瞬态；不能将三点当全BankMap、M2或时间最优模型。历史M1/M2及source/pose/nav/精确前视候选拒绝不变。

候选仅改变局部kinodynamic primitive的来源：从原V1多档主轴/副轴枚举，限制为当前必要覆盖验证的main+.5、rawencodedaux0/±30°三个forward点，并复用旧冻结BankMap对应点几何曲率（当前实测只核验、不拟合或作为未来标签）。unsupported点不返回几何、不外推/插值；原primitive arc .3、cost、root-cost、3000node、raster .870711、retention .6、参考更新、实际roll-PID/BankMap反馈/速度cap全部保持。只改变同一primitive来源消费环节，没有强制执行方向或提前换向。

CPU先验证supported/unsupported查询和原planner在原feature-off时输出自一致性；唯一frame140原R1已封存MODELmap/estimatedroot/goal原query preflight，检查原route+candidate nominal geometry及所有waypoint命令确在三个点内。不用future/GT选择；aftersealed作true clearance诊断。无path仅拒绝，不扫node/primitive长度/曲率范围。

如CPU/preflight合理，必须first scoped review/fullCPU/源码配置封存后再实际，最多一个conditional原60秒新任务运行。它是候选完整运行而非严格同源B0 A/B，oldactualR1 source不同前缀不能用来宣称孤立因果收益；若再需要严格A/B，须另固定完整commonreference/源输入并封存，而不是隐瞒差异。先用已知pose诊断还是估计pose保持当前估计pose层、source/pose gate分别保留。实际通过需要原native+true XYZgoal<=.2/60秒、source95/area6/class95、pose RMS.05/peak.1/anglep902、no contacts/collision、trueprogress≥4/过障碍、MODEL不写measuredfree、全执行/视频与trueguard。原35不触发。未通过不重复同candidate、扩大参数或再拟合模型；即使通过也只conditional任务，不全域动态模型/正式视觉benchmark成功。
