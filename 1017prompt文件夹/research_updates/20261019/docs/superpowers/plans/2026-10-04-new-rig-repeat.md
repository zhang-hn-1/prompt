# New camera rig repeat implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans. Use the progress ledger in the independent experiment directory.

**Goal:** Repeat the prior perception and information-condition diagnostic with the corrected physical camera installation and new actual images.

**Architecture:** Copy only frozen measurement/controller code into a new experiment directory. Reuse read-only pinned model checkpoints; regenerate camera-dependent input and truth with live Isaac. Prior request blocks are replayed in actual physics for six development trajectories to isolate the camera change; eight navigation runs use the unchanged causal shared controller.

**Tech Stack:** existing Python/Isaac/PyTorch/NumPy/pytest/Matplotlib environment.

**Spec:** prior PROTOCOL.md and CONTROL_DIAGNOSTIC.md in visual_information_diagnostic_20261003_v1, user request to repeat after corrected camera position.

## Global constraints
- Rig (0.20, ±0.30, 0.05) m in base_link; original orientation/intrinsics and fixed_camera_fill_v1 illumination.
- Roll PID .4/.1/.7, original protocol/smoother/supervisor, physical_contact_v2 and 35 s navigation budget unchanged.
- Same two official models/configs, no learning or GT scale fitting; exact old 66 frame IDs for six development trajectories if capture succeeds.
- Near-obstacle gate unchanged: n≥100, ≥90% 3D errors≤0.30m, p90≤0.30m for each critical subgroup.
- Offline models read RGB+K only; truth saved separately and opened only for scoring.
- Preserve old reports/results; newly acquired familiar trajectories are development data, not new held-out scenes.
- No merge/push/commit requested. Independent experiment directory provides artifact isolation; existing project worktree reused read-only.

## Review focus
- Old mask or optical translations accidentally retained in new captures/scoring.
- Failed simulator child exits 0 without required artifacts.
- Future commands/GT leaked into model or visible online policy.
- Photos reused rather than genuinely rendered from new rig.
- Hardcoded plot/report labels conceal changed outcomes or denominator changes.

### Task 1: freeze adaptation and test
- [x] Copy unchanged measurement/policy/map/tests; adapt only resource paths/version tags.
- [x] Test capture validation rejects old translations/mask identities and empty/unsynchronized captures before implement; then pass.
- [x] Assert unchanged copied controller/measurement source hashes and new rig identity.

### Task 2: real current-rig data and runs
- [x] Create immutable six-sequence old request manifests and 66 selected frame IDs.
- [x] Execute live replay captures (single reset per sequence, no per-frame state teleport).
- [x] Complete eight same-controller online map/visible runs and save synchronized RGB/depth, contacts, physics, videos.
- [x] Validate every child by both exit code and required artifacts; fail loudly otherwise.

### Task 3: fixed perception evaluation and independent comparison
- [x] Infer both pinned models on new 66 pairs with GPU isolated from Isaac.
- [x] Score with new mask and measured camera world pose, same gate and invalid denominator.
- [x] Analyze navigation/initial states/coverage; run unchanged CPU tests plus adaptation tests.
- [x] Save old/new tables, measured plots, complete evidence and separate perception/navigation conclusions.
- [x] Fresh whole-stage reviewer checks source, model, data identity and result claims; no material issues found.
