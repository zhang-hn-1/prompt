# 论文指标表索引

日期：2026-11-13

所有表格都是 3 个种子（42 / 2024 / 3407）的 **mean ± std**，数值单位为 %。表格已经带 `\resizebox{\textwidth}{!}`，可直接 `\input` 到论文里；如果期刊是单栏，建议改成 `table` 或拆成两张。

## 1. 记录级连续判决

| 文件 | 内容 | 建议位置 |
|---|---|---|
| `results/record_level_continuous/record_level_k4_metrics.tex` | 275 条 ≥4 段记录，第 4 段“当前片段” vs “4 段累计”；L-RDTNet / R(2+1)D-18 / R18-Transformer；AC、Macro-P、Macro-R、Macro-F1、MCC、AUC、鸟召回、空飘球召回 | 正文主表 |
| `results/record_level_continuous/record_level_cumulative_metrics.tex` | 同上 3 个模型，累计判决随 1/2/3/4 段变化；AC、Macro-P、Macro-R、Macro-F1、MCC、鸟/空飘球召回 | 正文或补充材料 |
| `results/record_level_continuous/record_level_full_metrics_per_seed.csv` | 11 个模型 × 3 种子 × 4 段 × 当前/累计，全部指标 + 四类各自 P/R/F1 + AUC | 数据备份 |
| `results/record_level_continuous/record_level_full_metrics_mean_std.csv` | 上表的 3 种子 mean/std | 数据备份 |

## 2. 短观测 / 帧顺序

| 文件 | 内容 | 建议位置 |
|---|---|---|
| `results/frame_diagnostic/short_observation_metrics.tex` | L-RDTNet：≥3 帧队列的 1/2/3 帧，≥6 帧队列的重复首帧/原序 6 帧/固定乱序 6 帧；AC、Macro-P、Macro-R、Macro-F1、MCC、R0–R3（R2 鸟、R3 空飘球） | 正文主表 |
| `results/frame_diagnostic/short_observation_full_metrics_per_seed.csv` | 上表逐种子数据 + 四类 P/R/F1 | 数据备份 |
| `results/frame_diagnostic/short_observation_full_metrics_mean_std.csv` | 上表 3 种子 mean/std | 数据备份 |

说明：短观测的预测文件只保存了 hard prediction，所以这张表没有 AUC。需要 AUC 的话，要改 `diagnose_frames.py` 保存概率后重跑三个种子（约 3 分钟）。

## 3. 19 条件鲁棒性

每张表都是 19 个条件 × 11 个模型，数值为 3 种子 mean ± std：

| 文件 | 指标 |
|---|---|
| `robustness_ext_table_accuracy.tex` | Accuracy |
| `robustness_ext_table_macro_precision.tex` | Macro-precision |
| `robustness_ext_table_macro_recall.tex` | Macro-recall |
| `robustness_ext_table_macro_f1.tex` | Macro-F1（19 条件完整版） |
| `robustness_ext_table_mcc.tex` | MCC |
| `robustness_ext_table_bird_recall.tex` | 鸟召回（R2） |
| `robustness_ext_table_balloon_recall.tex` | 空飘球召回（R3） |

逐种子、四类 P/R/F1 和 AUC 在：

- `robustness_ext_full_metrics_per_seed.csv`（627 行 = 33 model-seed × 19 条件）
- `robustness_ext_full_metrics_mean_std.csv`（209 行 = 11 模型 × 19 条件）

## 4. 复现

```bash
cd radar-autodl

python compute_record_level_metrics.py
python compute_frame_diagnostic_metrics.py
python compute_robustness_full_metrics.py
```

前置文件：

- 记录级：`results/robustness_ext/<model>_seed<seed>/clean.csv`
- 短观测：`results/frame_diagnostic/seed*/predictions.csv`
- 鲁棒性：`results/robustness_ext/<model>_seed<seed>/<condition>.csv`

## 5. 效率表（2026-11-15 新增）

| 文件 | 内容 |
|---|---|
| `results/efficiency/model_efficiency_table.tex` | 11 个模型 × 14 项指标：Params、Size、MACs、FLOPs、GPU FPS、GPU 延迟、峰值显存、CPU 延迟、AC、Macro-P、Macro-R、Macro-F1、MCC、AUC |
| `results/efficiency/model_efficiency_per_model.csv` | 逐模型数值 |

详见 `README_EFFICIENCY_AND_CAM.md`。硬件为 RTX 4070 Laptop + i9-14900HX；RK3588 数字需另测，不要与本表混排。

## 6. 鲁棒性样式曲线图（2026-11-15 新增）

仿照论文既有折线图版式重绘：R18-GRU（浅蓝空心圆）、R18-Transformer（蓝空心三角）、R(2+1)D-18（浅橙空心菱形）、**L-RDTNet(Ours)**（红实心方块粗线），Times 衬线字体，3 种子均值，无误差棒。文件在 `results/robustness_style/`：

| 文件 | x 轴 |
|---|---|
| `robustness_style_snr.png/.pdf` | Clean → 30/25/20/18/15/10 dB |
| `robustness_style_combined.png/.pdf` | Clean → Very light / Light / Medium / Heavy |
| `robustness_style_weak_global.png/.pdf` | Clean → 全局衰减 0.9/0.8/0.7/0.6 |
| `robustness_style_weak_peak.png/.pdf` | Clean → 峰值衰减 0.9/0.8/0.7/0.6 |
| `robustness_style_all.png/.pdf` | 19 条件全景（x 轴缩写标签） |

重新生成：`python robustness_style_figures.py`。

## 7. 写论文时的两个提醒

- 记录级表里 L-RDTNet 的累计 F1 最高，但它相对“首片段”的增益不显著；正文应写“累计判决显著优于只看当前片段”，不要写“显著优于只看第一段”。
- 短观测表里 2 帧的 Macro-F1 最高，3 帧的鸟召回更高、空飘球召回更低；正文应把它写成类别权衡，不要写“帧数越多越好”。
