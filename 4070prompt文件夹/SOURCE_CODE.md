# 导航源码包：服务器 AI 从这里开始

本目录包含两种不同用途的压缩包：

| 文件 | 用途 |
|---|---|
| `Spherical-robot-navigation-codex-navigation-core-update-20261008.zip` | 导航源码归档；恢复 baseline 工程时先检查它 |
| `navigation-baseline-handoff-20261016.zip` | 技术设计、实施计划和提示词；不包含导航源码 |

[下载导航源码归档](./Spherical-robot-navigation-codex-navigation-core-update-20261008.zip)

## 在服务器解压

以下命令从 `prompt` 仓库根目录执行。已有 checkout 先拉取最新提交；没有 checkout 时先克隆 `https://github.com/zhang-hn-1/prompt.git`。

```bash
git pull --ff-only
sha256sum "4070prompt文件夹/Spherical-robot-navigation-codex-navigation-core-update-20261008.zip"
unzip -n "4070prompt文件夹/Spherical-robot-navigation-codex-navigation-core-update-20261008.zip" -d source_archive_20261008
```

文件大小：18,612,191 bytes。

SHA256：`d3a1b6c6a426fe653312696bae3dca1f70f7704b4478ec4e34c999afe184246f`。

`unzip -n` 不覆盖已有文件；如同名解压目录中已有不同内容，请使用新的空目录，不能混合不同版本的归档。

解压后导航包位置：

```text
source_archive_20261008/
  Spherical-robot-navigation-codex-navigation-core-update-20261008/
    research/ball_wm/
      benchmark/policies.py
      benchmark/isaac_runtime.py
      benchmark/kinodynamic_planning.py
      benchmark/dynamic_lattice_planning.py
      scripts/collection_schema.py
      scripts/audit_dataset.py
      scripts/run_pointgoal_benchmark.py
      configs/data_contract.yaml
    research_additions/experiments/
      visual_information_new_rig_20261004_v2/model_inference.py
```

`model_inference.py` 包含 DA3 Metric 和 UniDepth V2 的左右图独立推理代码。请检查其完整依赖、预处理、权重路径和实际输出单位，不单独复制一个文件后就假定可运行。

## 恢复边界

这是 2026-10-08 的研究源码归档，不是已经验证能在当前服务器一键运行的发行版。外部模型权重、第三方依赖、场景资产和旧绝对路径仍需实际核对。

归档导航 runtime 使用 Isaac Sim；另一个 `legged_gym` 工程使用 Isaac Gym。先确认服务器环境和选定工程，不直接混合两套执行、相机和接触 API。

先阅读同目录 `design.md`、`implementation_plan.md`，完成归档与依赖清单，恢复最小 Oracle 运行，再按计划实施。拟新增的接口与已经存在但尚未解压的源码要分别列出，不能把当前目录搜索不到统一判断为需要重写。

原 `package_manifest.json` 校验文档交接包中的五份 Markdown；本源码 ZIP 的校验值以本文件为准。
