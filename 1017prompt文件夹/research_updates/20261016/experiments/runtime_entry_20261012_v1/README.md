# 本机复现环境只读入口

正式入口为preflight_v2.py，默认requirements_v3.json。保留的旧preflight.py/v1/v2文件只作历史证据。

```text
/home/an/miniconda3/envs/ball_wm/bin/python /home/an/Documents/ChatGPT/球型机器人导航/experiments/runtime_entry_20261012_v1/preflight_v2.py
```

检查2236个身份，包含6项主资源；6个Python模块可发现。宿主RTX3060/驱动580.178.04可见，本次exit0；隔离环境GPU不可见exit3。exit0只表示这些准备检查通过，未启动Isaac、未加载模型、未验证USD间接引用。physics_launch_admitted始终false。9项CPU检查与独立审查已闭合。

仍依赖原绝对路径；机器人USD路径来自只读工作树的collect_data_dubins.py，不是任意另一份项目资源。本工具不是跨机器独立运行包，不授权重跑旧实验或恢复预算。
