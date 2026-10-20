# Git Ignore Configuration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Configure the root Git repository so source code, research documents, configuration, and robot USD assets are trackable while local datasets, model artifacts, runtime outputs, caches, and the existing nested `dino_wm` repository are excluded.

**Architecture:** A single repository-root `.gitignore` defines project-specific exclusions and standard Python/tooling exclusions. The nested `research/ball_wm/models/dino_wm/` checkout remains untouched and independent; no files are staged or committed.

**Tech Stack:** Git ignore patterns, Bash verification commands

## Global Constraints

- Preserve all existing project files and Git histories.
- Do not run `git add` or create a commit.
- Keep `research/ball_wm/models/dino_wm/` as an independent nested repository.
- Keep source files, Markdown documents, YAML configuration, and USD robot assets visible to the root repository.

---

### Task 1: Configure and verify root ignore rules

**Files:**
- Create: `.gitignore`

**Interfaces:**
- Consumes: Repository paths under `research/ball_wm/` and the existing nested `dino_wm` checkout.
- Produces: Root-level Git ignore behavior only; no runtime interface.

- [ ] **Step 1: Record the precondition**

Run:

```bash
test ! -e .gitignore
git status --short --branch
```

Expected: `.gitignore` does not exist, and `project/` plus `research/` are untracked.

- [ ] **Step 2: Create the minimal ignore configuration**

Create `.gitignore` with explicit groups for OS/editor files, Python caches and environments, logs and experiment tracking, model artifacts, generated datasets/videos, and `research/ball_wm/models/dino_wm/`.

- [ ] **Step 3: Verify ignored and retained paths**

Run:

```bash
git check-ignore -v research/ball_wm/data/videos/20260525_122850_left.mp4
git check-ignore -v research/ball_wm/data/trajectories_dubins_640x480/ep0001/states.npz
git check-ignore -v research/ball_wm/scripts/__pycache__
git check-ignore -v research/ball_wm/models/dino_wm
test -z "$(git check-ignore research/Ball_Nav/POINTGOAL_STRUCTURED_WORLD_MODEL_PROPOSAL.md || true)"
test -z "$(git check-ignore project/rl_env_0.4/rl_env/rotunbot_wheel_sim_model_0.4/Rotunbot_wheel.usd || true)"
git status --short --branch
```

Expected: the four generated/nested paths report matching rules; the research proposal and USD asset are not ignored; `.gitignore`, `docs/`, `project/`, and `research/` remain untracked because nothing is staged.

- [ ] **Step 4: Leave the repository uncommitted for user review**

Do not run `git add` or `git commit`. Report the resulting ignore boundary and the independent nested repository status.
