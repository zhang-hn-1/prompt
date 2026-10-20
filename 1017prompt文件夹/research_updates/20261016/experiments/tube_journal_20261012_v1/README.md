# 执行前落盘派发协议

独占创建文件，同步目录及完整commit；每次intent落盘后才调用消费回调，返回后同步ack。异常关闭，拒绝重入与非owner线程。回调返回只证明回调返回，不证明物理执行；崩溃在intent与ack之间表示未知，不得恢复重派。没有真实sensor/request桥，没有物理采集准入。

source_mapping.json纠正现有位姿来源：native IMU+滚动模型+初始规范与地面先验，非RGB里程计。双RGB用于场景风险几何。原导航反馈请求不满足25步前瞻提交，不能用事后未来命令制作在线输入。

运行 `/home/an/miniconda3/envs/ball_wm/bin/python -m pytest -q experiments/tube_journal_20261012_v1/test_journal.py`。不防Python调用者直接改内部字段；没有可信认证或恢复。
