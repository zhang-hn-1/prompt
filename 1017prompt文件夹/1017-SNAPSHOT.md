# Spherical-robot-navigation 源码快照 · 2026-10-19 版

本目录是 `Spherical-robot-navigation` 项目的**完整源码目录快照**（直接以目录树存放，服务器上 clone 后无需解压）。注意：目录名 `1017prompt文件夹` 是交接包的命名，**实际内容是 2026-10-19 分支归档**，覆盖 10-17 至 10-19 的工作。

## 快照信息

| 项目 | 值 |
|---|---|
| 源仓库 | `git@github.com:Tremoring/Spherical-robot-navigation.git` |
| 分支 | `codex/navigation-research-update-20261019` |
| 源提交 | `c9cf9b9e0540133556027cb4b6bcacac96c7834b` |
| 提交说明 | `Archive four-camera baseline and memory research through v124` |
| 提交时间 | 2026-10-19 19:52:22 +0800 |
| 覆盖范围 | 2026-10-17 ~ 2026-10-19，研究归档截至 v124 |
| 上一版基线 | `8258abb`（对应 `research_updates/20261016`） |
| 文件数 | 11,685 |
| 体积 | 约 452 MB（git 对象口径），磁盘占用约 484 MB |

## 目录结构

```text
1017prompt文件夹/
  README.md                  源仓库说明
  RESEARCH_STATUS.md         当前研究状态（写于 10-16 时点，指向 20261016 快照）
  docs/                      benchmark 报告与 superpowers 计划、规格
  project/rl_env_0.4/        rotunbot_wheel USD 资产与仿真模型（约 54 MB）
  research/                  Ball_Nav 提案、ball_wm benchmark 实现与配置
  research_additions/        早期增量实验（DA3 Metric / UniDepth V2 左右图推理等）
  research_updates/
    20261016/                10-16 快照（上一版基线）
    20261019/                本次快照：configs / docs / experiments / reports
      README.md              本次快照的完整说明与结果表
      omitted_files.json     未上传文件的路径、大小与原因
      publication_manifest.json      收录文件的 SHA 映射
      publication_validation.json
  资料_球型机器人导航_2026-09-30/   项目现状、世界模型学习指南、相机安装核验等中文材料
  .gitignore
```

体积主要来源：`research_updates`（418 MB，大量 `policy_records.json`、`prediction_seal.json` 等逐点实验记录）+ `project`（54 MB USD 资产）。

文件构成：JSON 5,866、Python 4,331、Markdown 1,273、PNG 159、USD/USDA 9、YAML 12 等。

## 与 `4070prompt文件夹` 的关系

| 目录 | 内容 |
|---|---|
| `4070prompt文件夹` | 2026-10-08 导航源码 zip + 10-16 设计、实施计划与提示词（交接文档） |
| `1017prompt文件夹`（本目录） | 2026-10-19 完整源码与实验结果树，未压缩 |

## 使用方式

```bash
git pull --ff-only
cd "1017prompt文件夹"
less research_updates/20261019/README.md      # 先读这一份
```

## 边界说明

- **不含 `.git` 历史**。本目录无法 `git log`/`git diff` 追溯上游提交；需要历史请直接 clone 源仓库并切到 `codex/navigation-research-update-20261019`。
- **不是脱离环境的一键复现包**。原始 RGB/深度/IMU 大数组、逐帧二进制预测、视频、模型权重、Isaac 环境与外部依赖均保留在本地，未随本目录上传；未收录清单见 `research_updates/20261019/omitted_files.json`。脚本与科学 JSON 中的原绝对路径未改。
- **不作为验证通过的声明**。本次归档没有重跑仿真、推理、训练或科学评分，发布核验只检查文件身份、JSON 可读性、收录范围与 Markdown 链接转换。
- **研究状态**：`research/updates` 快照自述「可靠导航、有效学习导航增量与整体研究尚未完成，当前无准入新候选，任务为 PAUSED，无实验运行」。
- 源仓库 `.gitignore` 中列出的忽略项（如 `research/ball_wm/models/dino_wm/`、部分生成数据与权重）在源分支中同样未提交，因此本目录中也不存在。
- `project/rl_env_0.4/.../Rotunbot_wheel_base.usd` 为 53.3 MB，超过 GitHub 推荐的 50 MB 单文件上限（未超 100 MB 硬限）；后续再放大体积资产建议启用 git-lfs。
