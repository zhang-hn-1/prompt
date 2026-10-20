#!/usr/bin/env bash
set -euo pipefail

base='/home/an/Documents/ChatGPT/球型机器人导航/experiments/fixed_rig_visual_pilot_20261001/vision_detour_feasibility_v1'
repo='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm'

for episode in hard_0_e1_0003 hard_0_e1_0004; do
  PYTHONPATH="$repo:$base" /home/an/miniconda3/envs/ball_wm/bin/python "$base/run_continuous.py" \
    --manifest "$repo/configs/benchmark_v1/hard_0_dev_manifest.json" \
    --split dev --episode-id "$episode" --policy goal_greedy \
    --scene-root /home/an/project/NavDP/assets/scenes \
    --data-root "$repo/data" --run-root "$base/$episode" \
    --lighting-profile fixed_camera_fill_v1 --headless --record-video \
    > "$base/$episode.log" 2>&1
done
