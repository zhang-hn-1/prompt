# 论文交付包（2026-11-16）

所有图、表、数据都已统一配色、统一命名，可直接拷进论文工程。单位除注明外均为 %；准确性指标为 3 个种子（42 / 2024 / 3407）的 mean ± std。

## 1. 目录结构

```
paper_delivery_20261116/
  00_README.md                  ← 本文件
  01_figures/                   ← 论文用图（PNG 300dpi + PDF 矢量）
  02_tables/                    ← 可直接 \input 的 LaTeX 表
  03_data/                      ← 逐种子 / mean-std 数据 CSV
  04_notes/                     ← 表格索引、效率与 CAM 说明、关键数字速查
  MANIFEST.sha256               ← 全部文件校验和
```

## 2. 图 → 论文位置

| 文件 | 内容 | 建议位置 |
|---|---|---|
| `fig0_table3_comparison` | Table 3 完整对比表的渲染图（PNG+PDF） | 主结果表 |
| `fig1_robustness_snr` | Clean + SNR 30/25/20/18/15/10 dB，4 条模型曲线 | 4.4 鲁棒性主图 |
| `fig2_robustness_combined` | Clean + combined very light/light/medium/heavy | 4.4 鲁棒性 |
| `fig3_robustness_weak_global` | Clean + 全局衰减 0.9/0.8/0.7/0.6 | 4.4 鲁棒性 / 弱目标分析 |
| `fig4_robustness_weak_peak` | Clean + 峰值衰减 0.9/0.8/0.7/0.6 | 4.4 鲁棒性 / 弱目标分析 |
| `fig5_record_level_continuous` | 已观测片段数 vs Macro-F1（current vs cumulative） | 新增“连续判决”小节 |
| `fig6_short_observation` | 1/2/3 帧 + 6 帧原序/乱序 | 新增“短观测/帧顺序”小节 |
| `fig7_confusion_matrix` | 6 模型 2×3 行归一化混淆矩阵 | 主结果/消融前 |
| `fig8a`–`fig8d` | 4 模型 Grad-CAM（UAV-L / UAV-S / 鸟 / 空飘球） | 可解释性小节 |
| `09_supplementary/figS1` | 19 条件全景折线 | 补充材料 |
| `09_supplementary/figS2` | 11 模型 × 3 面板（SNR/combined/weak） | 补充材料 |
| `09_supplementary/figS3` | L-RDTNet vs R18-TAvg/R3D/MC3/GRU/XGB 聚焦图 | 补充材料 |
| `09_supplementary/figS4` | 10 模型 CAM 总网格 | 补充材料 |

配色全包统一：L-RDTNet 红实心方块、R(2+1)D-18 浅橙空心菱形、R18-Transformer 蓝空心三角、R18-GRU 浅蓝空心圆；混淆矩阵色系与折线图一一对应。

## 3. 表 → 论文位置

| 文件 | 内容 | 建议位置 |
|---|---|---|
| `tab1_robustness_accuracy` | 19 条件 × 11 模型 Accuracy | 正文或补充 |
| `tab2_robustness_macro_precision` | 同上 Macro-P | 补充 |
| `tab3_robustness_macro_recall` | 同上 Macro-R | 补充 |
| `tab4_robustness_macro_f1` | 同上 Macro-F1（完整 19 行） | 正文主表 |
| `tab5_robustness_mcc` | 同上 MCC | 正文或补充 |
| `tab6_robustness_bird_recall` | 鸟召回（R2） | 弱目标分析 |
| `tab7_robustness_balloon_recall` | 空飘球召回（R3） | 弱目标分析 |
| `tab8_record_level_k4` | 275 条记录第 4 段：current vs cumulative，AC/P/R/F1/MCC/AUC/鸟/空飘球 | 连续判决主表 |
| `tab9_record_level_cumulative` | 累计判决 1/2/3/4 段 | 连续判决补充 |
| `tab10_short_observation` | 1/2/3 帧 + 6 帧原序/乱序，AC/P/R/F1/MCC/R0–R3 | 短观测主表 |
| `tab11_efficiency` | 11 模型 × 14 项：Params/Size/MACs/FLOPs/GPU/CPU/AC/P/R/F1/MCC/AUC | 轻量化/推理小节 |
| `tab12_table3_comparison` | Table 3：20 个基线与 L-RDTNet 的 AC/P/R/F1/MCC/AUC/Params 对比 | 主结果表 |

表格均带 `\resizebox{\textwidth}{!}`，双栏期刊可直接 `\input{...}`；单栏需改成 `table` 或拆分。

## 4. 关键数字速查（写正文用）

**Clean / 温和条件**
- Clean Macro-F1：L-RDTNet **92.37**，R18-Transformer 90.98，R18-GRU 90.79，R18-LSTM 90.53，R(2+1)D-18 90.18，Handcrafted+XGB 82.99。
- Table 3 新增基线：C3D 80.25、Swin-Tiny 85.85、ResNet18-Frame Feature Avg. 89.31（单种子）、SVM 39.70、RF 82.43。
- L-RDTNet 在 19 个条件中 **9 个第一**：Clean、SNR30/25/20、Comb very light/light、WG0.9/0.8、WP0.9。

**强扰动（必须如实写）**
- L-RDTNet 低于 70 的 4 条：SNR10 **49.41**、Combined heavy **26.66**、WG0.6 **47.58**、WP0.6 **68.92**。
- SNR10 最优是 ResNet18-TAvg **84.25**；Combined heavy 最优是 MC3-18 **49.95**；WG0.6 最优是 ResNet18-TAvg **69.38**；WP0.6 最优是 MC3-18 **79.85**。
- 结论只能写“clean 和温和扰动下最强，强噪声/强复合扰动下退化更快”，不能写“全面最鲁棒”。

**记录级连续判决（275 条 ≥4 段记录）**
- L-RDTNet：首片段 93.49 → 4 段累计 **95.74**；累计−当前 **+3.19 [1.43, 5.25]**；累计−首片段 +2.25 **[−0.15, 4.84]（不显著）**。
- R(2+1)D-18：累计 94.15，累计−首片段 **+5.33 [2.30, 8.57]**（显著）。
- R18-Transformer：累计 94.73，累计−首片段 **+3.67 [1.14, 6.33]**（显著）。

**短观测（L-RDTNet）**
- ≥3 帧 4,489 条：1/2/3 帧 Macro-F1 = **75.67 / 85.46 / 84.64**；2−1 **+9.79 [7.98, 11.80]**；3−2 **−0.82 [−3.27, 1.71]（不显著）**。
- 逐类召回：鸟 61.2→77.7→**88.0**；空飘球 94.3→92.7→**84.7**（3 帧是类别权衡）。
- ≥6 帧 848 条：重复首帧 50.74、原序 **82.36**、乱序 79.32；原序−乱序 **+3.04 [1.11, 5.00]**。

**效率（RTX 4070 Laptop + i9-14900HX，batch 32）**
- L-RDTNet：Params **8.37 M**、MACs 8.47 G、GPU **1.82 ms**/样本（549 FPS）、CPU 17.4 ms、峰值显存 470 MB。
- R(2+1)D-18：31.30 M、5.28 ms、90.18 F1；R18-Transformer：17.48 M、**1.61 G**、0.55 ms、90.98 F1。
- 写“参数量最小且精度最高”，不要写“计算量最小”——L-RDTNet 的 MACs 高于轻量 2D/时序基线。

## 5. 使用提醒

1. **RK3588 数字不在此包**：表中 CPU 延迟是 i9-14900HX；论文如保留 RK3588 行，需单独标注硬件，不能与本表混排。
2. **FLOPs 口径**：表中 FLOPs = 2×MACs；若期刊用 MACs 口径，直接引用 CSV 的 `macs_g`。
3. **短观测无 AUC**：诊断脚本只存了 hard prediction；如需 AUC 需保存概率后重跑（约 3 分钟）。
4. **M06 Frame Feature Average 只有 seed42**：混淆矩阵里该面板是单种子，其余 5 个模型是 3 种子均值。
5. **CAM 是解释性可视化**，图注建议 “Grad-CAM on the last convolutional stage”；不要写成注意力机制的直接证据。
6. **摘要数字**：5,194 条测试序列来自 **277 条留出记录**；**1,384 条是全数据集记录数**。若 IEEE Access 摘要仍写成“1,384 条测试记录”，务必改。
7. **Table 3 的 4 个空行**：ViT-B/16、Swin-S、TimeSformer-Base、Video Swin-S 在本机备份中没有冻结划分的 checkpoint。实测从零训练约 3 / 3 / 9.4 / 1.3 分钟每 epoch（8GB 笔记本 GPU），40 epoch 单种子需 1–6 小时，三件套全部跑完需十几小时，因此标为 `--`。ResNet18-LSTM 已补训 3 个种子（90.53 Macro-F1）。若提供 AutoDL/服务器算力，配置已备好，可直接开跑。

## 6. 完整原始预测

本包只放论文用图表和汇总数据。逐条件预测（627 个 CSV）与逐样本帧诊断预测在：

- 项目根目录 `robustness_ext_results.zip`（138 MB）
- 项目根目录 `continuous_and_frame_experiments.zip`（446 KB）

复现脚本：`compute_record_level_metrics.py`、`compute_frame_diagnostic_metrics.py`、`compute_robustness_full_metrics.py`、`compute_efficiency_metrics.py`、`gradcam_visualization.py`、`cam_style_figure.py`、`confusion_matrix_style.py`、`robustness_style_figures.py`。
