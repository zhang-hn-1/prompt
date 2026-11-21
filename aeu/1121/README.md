# 1121 — L-RDTNet comparison figures

Two qualitative comparison figures generated on 2026-11-21. Both use the
frozen test split and the L-RDTNet seed-42 checkpoint unless otherwise noted.

## 1. Model comparison (final-conv Grad-CAM)

Files: `suite_model_cam_comparison.png/.pdf/.svg`

- Rows: UAV-L, UAV-S, Bird, Balloon.
- Columns: raw RD mean and final-convolution Grad-CAM of ResNet18-GRU,
  R18-Transformer, R(2+1)D-18 and L-RDTNet.
- The four clips are correctly classified by **all four** models:

| Class | Sample | Minimum true-class probability |
|---|---|---:|
| UAV-L | `113_6` | 0.969 |
| UAV-S | `302_15` | 0.967 |
| Bird | `522_13` | 0.968 |
| Balloon | `2995_3` | 0.949 |

CAMs are averaged over six frames and normalized per panel, so the comparison
is about spatial focus rather than amplitude. The baselines produce broad
Doppler-band responses; L-RDTNet is localized near the target peak. Note that
the CAM layer is the final convolutional stage: for the GRU/Transformer models
the temporal module comes afterwards.

## 2. Progressive encoder-ablation t-SNE

Files: `suite_ablation_tsne.png/.pdf/.svg`

275 test records, features averaged over the first four segments, standardized
and embedded with t-SNE (perplexity 30). Features are from the seed-42
checkpoints; the Macro-F1 values are three-seed means from the record-level
cumulative ablation table.

| Panel | Params (M) | Macro-F1 (%) | 4-class silhouette |
|---|---:|---:|---:|
| MS-RadarNet-Stem | 0.05 | 55.20 | 0.015 |
| MS-RadarNet-LocalRD | 0.49 | 84.67 | 0.202 |
| MS-RadarNet-Motion | 2.06 | 91.08 | 0.328 |
| L-RDTNet | 8.37 | 95.74 | 0.653 |

The silhouette progression mirrors the F1 progression: the stem features are
nearly inseparable, LocalRD and Motion progressively form class structure, and
L-RDTNet produces compact per-class clusters.

## Metadata and captions

- `suite_model_cam_comparison_meta.json`: selected samples, model predictions
  and minimum probabilities.
- `suite_ablation_tsne_meta.json`: panel metrics and silhouette scores.
- `captions.tex`: ready-to-paste LaTeX captions (adjust the graphics path).
