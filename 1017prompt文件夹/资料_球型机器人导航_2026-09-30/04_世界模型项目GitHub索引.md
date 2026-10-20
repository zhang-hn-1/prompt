# 世界模型与视觉导航项目 GitHub 索引

核对日期：2026-09-30。这份索引补充[世界模型学习指南](02_世界模型学习指南.md)：论文用于理解方法，作者项目页适合看演示，GitHub 用于查看公开项目。这里**只提供入口**，不复制源码，也不把其他机器人上的结果当成本球型机器人的成绩。

## 优先看：与本项目最接近

| 项目 | GitHub 入口 | 作者项目或论文 | 建议关注的研究问题 |
|---|---|---|---|
| Navigation World Models（NWM） | [facebookresearch/nwm](https://github.com/facebookresearch/nwm) | [作者演示](https://www.amirbar.net/nwm/) | 能否用预测的未来第一视角画面为候选导航轨迹排序？官方仓库公开，但大规模视频模型的计算需求与本项目不同。 |
| DINO-WM | [gaoyuezhou/dino_wm](https://github.com/gaoyuezhou/dino_wm) | [作者演示](https://dino-wm.github.io/) | 预测视觉特征而非逐像素重建，是否还能保留墙、走廊和障碍信息？ |
| V-JEPA 2 / V-JEPA 2-AC | [facebookresearch/vjepa2](https://github.com/facebookresearch/vjepa2) | [Meta 项目介绍](https://ai.meta.com/research/vjepa/) | 视频表征和**动作条件**预测如何分开建立、再连接？论文的机器人任务主要是操作，不是本项目导航。 |
| Mobile World Models（MWM，2026 预印本） | [AIGeeksGroup/MWM](https://github.com/AIGeeksGroup/MWM) | [论文](https://arxiv.org/abs/2603.07799) | 多步预测如何保持与给定动作一致？仓库已公开，但其 README 仍有未勾选的发布事项，阅读前不要假定所有资源齐备。 |
| NavWAM（2026 预印本） | **暂无作者公布的 GitHub 仓库** | [作者项目页](https://dachii-azm.github.io/navwam/) | 如何把未来视野、目标进度和动作预测联合用于闭环？作者页当前写着“Code coming soon”，这里不提供第三方冒名实现。 |

## 作为方法对照

| 项目 | GitHub 入口 | 阅读价值与注意点 |
|---|---|---|
| TD-MPC2 | [nicklashansen/tdmpc2](https://github.com/nicklashansen/tdmpc2) | 学模型预测与短时动作选择怎样结合；其连续控制任务与本球机的相机遮挡/转向执行不同。 |
| DreamerV3 | [danijar/dreamerv3](https://github.com/danijar/dreamerv3) | 学想象轨迹训练策略；这是作者维护的**重实现**，仓库声明与 Google/DeepMind 无关。 |
| Occupancy Anticipation | [facebookresearch/OccupancyAnticipation](https://github.com/facebookresearch/OccupancyAnticipation) | 学“已观测/未观测/预测占据”的区别；原工作使用 RGB-D，本项目在线只有固定原位双目 RGB。 |
| PlaNet | [google-research/planet](https://github.com/google-research/planet) | 经典潜空间规划入口；仓库已归档为只读，适合看方法，不应默认能直接在当前环境运行。 |
| I-JEPA | [facebookresearch/ijepa](https://github.com/facebookresearch/ijepa) | 看静态图像表征学习；仓库已归档，而且 I-JEPA 本身不是动作条件导航模型。 |

对师弟最值得先打开的是 **NWM → DINO-WM → V-JEPA 2**：它们分别回答“预测未来视野如何帮助选路”“能否用视觉特征预测而不还原所有像素”“如何从视频表征走向动作条件预测”。之后再看 MWM 的多步一致性和 NavWAM 的闭环动作问题。公开 GitHub 说明有学习入口，**不说明这些模型能直接替换本项目的 Oracle、固定相机视觉基线或当前世界模型**。
