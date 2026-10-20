# 第一版隐藏无效：结果解释限制

2026-10-03，查看实际 RGB 后发现诊断目标球仍可见。真实 USD CPU 复现表明：此目录 run_worker 在 typeless 父节点上设置 invisible，子 Sphere/BasisCurves 仍 ComputeVisibility=inherited。目录名 clean_render_v1 与身份元数据中的隐藏声明不能作为成功证据。

本目录四条运行、原 SHA、评分及接口测试全部保留；不得称为清洁无 Oracle 图像提示的视觉运行。实际修复与逐次采样查询在 ../clean_render_v2。算法候选与所有控制参数不变，未追改本目录历史结果。
