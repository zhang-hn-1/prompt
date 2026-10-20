# 原生估计位姿与RGB锚点来源接口

provide调用已有FlatGroundPoseBridge.at精确时间，仅传估计XY、速度、航向及显式初始规范/地面先验。provide_clocked从同一runtime四元组取得relative/raw时间，读前后时钟不得推进，并复制640×480×3 uint8 RGB，保存包含时钟及SHA的receipt。8合成检查通过，独立复审闭合；历史一个锚点只核对schema/time，不是fresh学习数据。

仍未证明：类型与字段名不能认证原始IMU输入；camera renderingtime不能证明像素曝光内容新鲜；read_rgb回调实际来源与固定捕获须由真实runner核验。没有Isaac live运行。

collector_pilot_design_v1.json预冻结最多1 E0原35秒全零请求资格pilot：70×25raw=1750ticks=350原动作块。70窗口同episode，禁止训练/测试拆分。旧预算不重开、无网络/teacher/fit、重试0。它仅数据接口资格，不建立学习覆盖或导航效果。实际runner及独立启动前审查未完成，不能从该文件启动。

运行：`/home/an/miniconda3/envs/ball_wm/bin/python -m pytest -q experiments/tube_anchor_provider_20261012_v1/test_provider.py`。下一直接实现实际runner官方RGB读取/固定capture/nativeIMU来源桥，使用已完成BlockBridge和持久化提交协议，不再增加重复通用接口。
