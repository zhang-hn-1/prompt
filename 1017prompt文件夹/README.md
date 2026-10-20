# Spherical-robot-navigation 源码快照（2026-10-17）

本目录是 `Spherical-robot-navigation` 项目的完整源码快照，直接以目录树形式存放，服务器上无需解压，clone 后即可浏览和运行。

## 快照信息

| 项目 | 值 |
|---|---|
| 源仓库 | `git@github.com:Tremoring/Spherical-robot-navigation.git` |
| 分支 | `main` |
| 源提交 | `0b965f79685b2ee0ee3ede64bad2a15883290749` |
| 提交说明 | `feat: evaluate legacy world model on golden benchmark` |
| 提交时间 | 2026-09-18 11:41:28 +0800 |
| 快照日期 | 2026-10-17 |
| 文件数 | 184 |
| 体积 | 约 58 MB |

## 目录结构

```text
1017prompt文件夹/
  docs/                      benchmark 报告与 superpowers 计划、规格文档
    benchmark_golden_v1_20260917.md
    benchmark_world_model_legacy_20260918.md
    superpowers/plans/       按日期归档的实施计划
    superpowers/specs/       对应的设计与验收规格
  project/
    rl_env_0.4/rl_env/       rotunbot_wheel USD 资产与仿真模型（约 54 MB，体积主要来源）
  research/
    Ball_Nav/                研究提案与 refine 记录
    ball_wm/                 benchmark 实现
      benchmark/             规划器、策略、runner、轨迹跟踪与验收
      configs/               benchmark_v1、训练配置、数据契约
      data/                  benchmark_v1 场景、occupancy、corridor 数据
      scripts/               数据处理与审计脚本
      run.sh
  .gitignore
```

文件构成：Python 102、Markdown 46、YAML 9、JSON 9、USD/USDA 8、PNG/PNPY/NPZ 若干。

## 在服务器上使用

从 `prompt` 仓库根目录执行：

```bash
git pull --ff-only
cd "1017prompt文件夹"
git -C ../ log -1 --format=%H    # 确认拿到的是最新提交
```

## 边界说明

- 这是源码快照，**不含 `.git` 历史**，不能在此目录里直接执行 `git log`、`git diff` 追溯上游提交。需要完整历史时请直接 clone 源仓库。
- 其中 USD 资产、`.npz`/`.npy` 数据为仓库内已有文件，未附外部模型权重、第三方依赖和场景资产。
- 导航 runtime 依赖 Isaac Sim；`legged_gym` 工程依赖 Isaac Gym。两套执行、相机与接触 API 不要混用。
- 旧的绝对路径、权重路径与依赖版本仍需在实际服务器环境核对，不能假定开箱可运行。
