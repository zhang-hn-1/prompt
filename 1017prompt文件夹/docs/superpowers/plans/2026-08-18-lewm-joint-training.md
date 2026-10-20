# LeWM Joint World Model Training Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the LeWM JEPA into a stereo+proprioception joint world model that also predicts local BEV occupancy and supports PointGoal iCEM planning for the ball robot.

**Architecture:** Keep LeWM's encoder→predictor→SIGReg core and add (1) a stereo fusion encoder, (2) a proprioception encoder, (3) a BEV probe head supervised by `geometry_bev` and visibility masks, and (4) a standalone PyTorch trainer so we do not depend on `stable-pretraining`/`stable-worldmodel`/Lightning, which are absent from the `ball_wm` env. Data comes from the schema-1.1 collector output.

**Tech Stack:** Python 3.10, PyTorch 2.5.1, einops, Pillow, NumPy, SciPy, YAML. No Isaac Sim imports in training code.

## Global Constraints

- Data root is the collector output: `research/ball_wm/data/trajectories_dubins_v2_*` with `epNNNN/` directories.
- Stereo frame rate is 10 Hz; control/state rate is 50 Hz; action block per visual transition is exactly 5 applied actions (`action_block=5`).
- BEV is `uint8[80,80]`, classes `0=free, 1=occupied, 2=unknown`, base frame, image up=`+X`, image left=`+Y`.
- RGB is 640×480; training may resize, but the loader must keep aspect and document the resize.
- Proprioception model state is 17-D: base linear velocity (3), base angular velocity (3), gravity in base (3), joint position (3), joint velocity (3), previous action (2).
- PointGoal is not a model input; it is used by the planner as a relative base-frame goal.
- Depth and world pose are training/eval teachers only, never model deployment inputs.
- Training code must run in `/home/an/miniconda3/envs/ball_wm` without Isaac Sim.

---

### Task 1: Ball-WM episode dataset and statistics

**Files:**
- Create: `research/ball_wm/training/__init__.py`
- Create: `research/ball_wm/training/ball_wm_dataset.py`
- Create: `research/ball_wm/training/dataset_stats.py`
- Create: `research/ball_wm/tests/test_ball_wm_dataset.py`

**Interfaces:**
- Produces: `BallWMDataset(root, history_frames, num_preds, image_size, action_block, with_bev, normalize)`, `SampleConfig`, `make_train_val_test_split(dataset, ratios, seed, unseen_scene_test)`.
- Consumes: schema-1.1 episode directories.

- [ ] **Step 1: Write failing tests for shapes and action alignment**

Create a synthetic episode writer in the test that writes 2 episodes with 30 state rows and 6 stereo frames, then assert:

```python
def test_dataset_returns_contract_shapes(tmp_path):
    ds = BallWMDataset(tmp_path, history_frames=3, num_preds=1, image_size=(16, 16))
    sample = ds[0]
    assert sample["pixels_left"].shape == (4, 3, 16, 16)
    assert sample["pixels_right"].shape == (4, 3, 16, 16)
    assert sample["action"].shape == (4, 10)
    assert sample["proprio"].shape == (4, 17)
    assert sample["geometry_bev"].shape == (4, 80, 80)
```

and an alignment test that the first action block equals `applied_action[5:10]`.

- [ ] **Step 2: Run tests and verify RED**

Run: `pytest -q research/ball_wm/tests/test_ball_wm_dataset.py`

Expected: import failure because `ball_wm_dataset` does not exist.

- [ ] **Step 3: Implement episode discovery, frame windows, and proprioception**

`BallWMDataset.__init__` scans `root/ep*/metadata.json`, validates schema, precomputes valid `(episode_index, start_frame)` pairs where `start_frame + history_frames + num_preds <= num_frames`. `__getitem__` loads PNGs via PIL, resizes, applies ImageNet normalization when requested, extracts action blocks with `applied_action[5*(f+1):5*(f+2)]`, computes 17-D proprio, and loads BEV/mask PNGs as long/float tensors. Caching is a LRU on episode arrays to keep I/O low.

- [ ] **Step 4: Implement scene-aware deterministic split**

`make_train_val_test_split` groups by `metadata["scene"]`, shuffles scenes with a seed, and optionally reserves whole unseen scenes for test. Fallback to episode-level split when only one scene exists.

- [ ] **Step 5: Implement dataset_stats CLI**

`dataset_stats.py --root <path>` prints episode count, stereo frames, per-scene counts, action min/max, BEV class distribution, collision ratio, and per-episode file counts.

- [ ] **Step 6: Run full test suite and commit**

Run `pytest -q research/ball_wm/tests` and `git commit`.

---

### Task 2: Stereo/proprio joint LeWM model

**Files:**
- Create: `research/ball_wm/training/joint_jepa.py`
- Create: `research/ball_wm/training/joint_model.py`
- Create: `research/ball_wm/tests/test_joint_model.py`

**Interfaces:**
- Produces: `JointJEPA(encoder, predictor, action_encoder, proprio_encoder, bev_head, mask_head)`, forward returning `emb`, `act_emb`, `bev_logits`, `mask_logits`.

- [ ] Build stereo fusion: encode left/right with a shared or twin ViT, fuse patch/CLS embeddings (concat+proj or cross-attention).
- [ ] Proprio encoder: MLP from 17-D to embedding; add to context tokens.
- [ ] BEV head: transformer/MLP from latent `emb` to `(B,T,80,80)` class logits; mask head to `(B,T,80,80)` binary logits.
- [ ] Losses: LeWM pred loss + SIGReg + BEV cross-entropy + visibility BCE (+ optional state MSE).
- [ ] Unit tests with random tensors for shapes and forward/backward.

---

### Task 3: Standalone trainer

**Files:**
- Create: `research/ball_wm/training/train_joint.py`
- Create: `research/ball_wm/configs/train_joint.yaml`

**Interfaces:**
- Produces: `train_joint.py --config ... --data_root ...`, checkpoint callback, wandb/tensorboard optional.

- [ ] Implement epoch loop, AdamW + LinearWarmupCosineAnnealingLR, mixed precision.
- [ ] Implement sub-trajectory batching and `num_preds` target slicing.
- [ ] Implement checkpointing and resume.
- [ ] Run a 1-epoch smoke on the 5 collected episodes.

---

### Task 4: iCEM planner and evaluation

**Files:**
- Create: `research/ball_wm/training/planner.py`
- Create: `research/ball_wm/training/evaluate_pointgoal.py`

**Interfaces:**
- Produces: `plan(model, context, goal_base_xy, config) -> action_sequence`, success/collision evaluation script.

- [ ] Implement CEM sampling of 5-step action blocks, latent rollout, goal embedding cost + BEV occupancy cost.
- [ ] Evaluate success rate and collision rate on held-out episodes.

---

## Final verification

- [ ] `pytest -q research/ball_wm/tests` green.
- [ ] One training epoch runs on the 5-episode dataset and writes a checkpoint.
- [ ] Evaluation script reports success/collision on held-out episodes.
