# 效率基准 + Grad-CAM 注意力图

日期：2026-11-15

## 1. 效率/轻量化/准确性指标（11 个模型）

硬件：RTX 4070 Laptop GPU 8GB + Intel Core i9-14900HX（32 线程）。输入 `1×6×31×226`，batch 32。
注意：本机没有 RK3588；论文里如需边缘设备延迟，可用同一脚本在 RK3588 上复测 CPU 部分。

| 文件 | 内容 |
|---|---|
| `results/efficiency/model_efficiency_table.tex` | 论文主表：14 项指标 × 11 个模型 |
| `results/efficiency/model_efficiency_per_model.csv` | 逐模型数值 |

14 项指标：

- 轻量化：Params (M)、模型体积 Size (MB)、MACs (G)、FLOPs (G)（FLOPs = 2×MACs）
- 推理：GPU FPS、GPU 每样本延迟 (ms)、GPU 峰值显存 (MB)、CPU 每样本延迟 (ms)
- 准确性（clean，3 种子均值）：AC、Macro-P、Macro-R、Macro-F1、MCC、AUC

关键数字（节选）：

| 模型 | Params (M) | MACs (G) | GPU ms | CPU ms | Macro-F1 |
|---|---:|---:|---:|---:|---:|
| **L-RDTNet** | **8.37** | 8.47 | 1.82 | 17.4 | **92.37** |
| R(2+1)D-18 | 31.30 | 9.30 | 5.28 | 31.9 | 90.18 |
| R18-Transformer | 17.48 | **1.61** | **0.55** | **3.7** | 90.98 |
| ResNet18-Mid | 11.17 | **0.26** | **0.09** | **0.6** | 84.48 |
| ResNet18-TAvg | 11.17 | **0.26** | **0.09** | **0.6** | 88.20 |
| ResNet18-GRU | 12.16 | 1.58 | 0.50 | 3.6 | 90.79 |
| ResNet18-TCN | 14.32 | 1.60 | 0.51 | 3.7 | 89.81 |
| ResNet18-Attn | 11.17 | 1.58 | 0.49 | 3.6 | 89.55 |
| R3D-18 | 33.15 | 9.22 | 2.01 | 18.3 | 90.49 |
| MC3-18 | 11.47 | 9.30 | 2.24 | 21.4 | 90.03 |
| Handcrafted+XGB | – | – | – | – | 82.99 |

写论文的读法：L-RDTNet 是**参数量最小（8.37M）且准确率最高**的模型；单看 MACs，L-RDTNet（8.47G）高于 R18-Transformer（1.61G）和 ResNet 系 2D 模型（0.26G），GPU/CPU 延迟也因此高于它们——这正是"L-RDTNet 精度换算力"的诚实表述。R(2+1)D-18 参数量是 L-RDTNet 的 3.7 倍、精度反而更低。

Handcrafted+XGB 无神经网络结构，Params/MACs/GPU 列为"--"。

## 2. Grad-CAM 注意力图

Grad-CAM（目标层梯度加权），每类选 1 条代表性测试序列（≥3 帧），取 CAM 响应最强的帧叠加到灰度 RD 图上；低响应区域不显色。

| 文件 | 内容 |
|---|---|
| `results/gradcam/all_models_cam_grid.png/.pdf` | **10 行（模型）× 4 列（类别）对比大图**，论文用 |
| `results/gradcam/<model>/cam_grid.png` | 单模型 1×4 网格 |
| `results/gradcam/cam_meta.csv` | 每张图用的 sample_id、类别、峰值帧号 |

覆盖 10 个神经网络模型。Handcrafted+XGBoost 没有卷积特征图，不适用 CAM。

目测结论：

- **L-RDTNet 的激活最聚焦**：4 个类别都集中在目标航迹附近的一个紧凑区域。
- R(2+1)D-18 / R3D-18 / MC3-18 激活沿航迹分布但更分散。
- ResNet18-Mid / TAvg（单帧/均值图模型）呈竖直条带状激活，反映它们只看单帧或均值图。
- 时序模型（GRU/TCN/Attn/Transformer）激活偏带状且更分散，与它们的特征聚合方式一致。

选样和峰值帧号在 `cam_meta.csv`（如 L-RDTNet：UAV-L `4_2` 帧 1、UAV-S `302_0` 帧 2、Bird `281_1` 帧 2、Balloon `2872_0` 帧 1）。换样本只改 `representative_indices()`。

## 3. 论文版式 Grad-CAM 图（2026-11-15 新增）

仿照论文已有版式重新绘制：上行 **Overlay**（倒置 RD 图 viridis 底 + CAM 半透明叠加），下行 **CAM only**（magma），列序 R18-GRU → R(2+1)D-18 → R18-Transformer → **L-RDTNet**（加粗），y 轴 Range bin 0–30，x 轴 Doppler bin 0–226，colorbar 标 Grad-CAM，Times 衬线字体。

| 文件（`results/gradcam/style/`） | 内容 |
|---|---|
| `cam_style_4models_class{k}_{名称}_{样本}.png/.pdf` | 每类一张，4 列 = 论文四个模型，正文用 |
| `cam_style_10models_class{k}_{名称}_{样本}.png/.pdf` | 每类一张，10 列 = 全部神经网络模型，补充材料用 |

- CAM 取 6 帧的时间平均（gaussian σ=3 平滑），Overlay 底图取中间帧、倒置显示并做轻度平滑。
- 类别 0–3 的样本：UAV-L `4_2`、UAV-S `302_0`、Bird `281_1`、Balloon `2872_0`。
- 重新生成：`python cam_style_figure.py`（约 2 分钟）。

## 4. 复现

```bash
cd radar-autodl
python compute_efficiency_metrics.py      # ~2 分钟
python gradcam_visualization.py           # ~2 分钟
```

注意：GRU 的 CAM 在 `resnet18_gru` 分支临时禁用 cuDNN（cuDNN RNN 反向不允许 eval 模式），数值等价、只影响速度。

## 4. 局限与提醒

- 效率数字依赖本机硬件和 PyTorch 版本，比较是公平的（同一 GPU/CPU、同一 batch、同一输入尺寸）；但**不能直接与 RK3588 数字混排**，需要分开列或重测。
- GPU 计时只含前向（批次已预载内存），不含磁盘读图。
- FLOPs = 2×MACs 是常见口径；如果论文用 "MACs" 口径，直接用 MACs 列。
- Grad-CAM 是解释性可视化，不构成"注意力机制有效"的证明；正式 claims 建议配合消融实验。
