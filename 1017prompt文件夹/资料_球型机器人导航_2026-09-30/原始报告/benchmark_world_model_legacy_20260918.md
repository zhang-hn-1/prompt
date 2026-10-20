# Previous world-model checkpoints on PointGoal Benchmark V1

Date: 2026-09-18

These are diagnostic adapter runs on the frozen golden manifest. Both
checkpoints were trained on the 2026-08-28 Dubins dataset, whose metadata has
no action protocol field and is therefore classified by the evaluator as
`ball_action_v1_aux16`. Benchmark V1 requires `ball_action_v2_aux30` (the
auxiliary range changes from ±0.2809658115 rad to ±0.5235987756 rad). The
adapter searches inside the legacy range and emits actions accepted by the V2
runner. These numbers must not be presented as V2-trained policy results.

## Held-out model metrics

The deterministic split uses seed `3072`, holds out scenes `easy_0`, `easy_3`,
and `easy_8`, and evaluates 765 contexts.

| checkpoint | protocol | horizon-1 XY MAE | horizon-5 XY MAE | horizon-5 yaw MAE | horizon-5 occupied IoU |
| --- | --- | ---: | ---: | ---: | ---: |
| `a_pose/joint_wm_epoch09.ckpt` | `ball_action_v1_aux16` | 0.0574 m | 0.1147 m | 7.22° | 0.0000 |
| `b_spatial/joint_wm_epoch09.ckpt` | `ball_action_v1_aux16` | 0.0660 m | 0.1339 m | 9.18° | 0.0897 |

The b_spatial checkpoint predicts non-trivial occupancy, but its held-out
occupied IoU remains low and its predicted occupancy is over-dense.

## Golden E0/E1/E2 closed-loop results

All runs use the frozen `ball_pointgoal_v1_1_35s` protocol, the same manifest,
the same three episodes, CEM horizon 5 / 64 samples / 8 elites / 3 iterations,
and `perfect_localization` as required by the benchmark runner.

| adapter/checkpoint | E0 | E1 | E2 | success | collision | timeout | SPL |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: |
| a_pose epoch09, safety 0.0 | collision | collision | timeout | 0/3 | 2/3 | 1/3 | 0.000 |
| b_spatial epoch09, safety 0.1 | collision | collision | timeout | 0/3 | 2/3 | 1/3 | 0.000 |

The b_spatial run reduced E0 final distance from 15.82 m to 3.29 m and made
5.55 m of goal progress before collision. It did not solve E1 and moved away
from the E2 goal until timeout. No episode had a runtime error.

Run artifacts are kept in:

- `research/ball_wm/runs/benchmark_golden_world_model_legacy_apose_epoch09/`
- `research/ball_wm/runs/benchmark_golden_world_model_legacy_bspatial_epoch09_safety01/`

The runner now exposes the explicitly labelled `world_model` policy and
requires `--allow-legacy-checkpoint` before accepting a legacy checkpoint.
