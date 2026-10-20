#!/usr/bin/env bash
set -euo pipefail

base='/home/an/Documents/ChatGPT/球型机器人导航/experiments/fixed_rig_visual_pilot_20261001/illuminated_replay_v1'
repo='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm'
worker='/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_vertical_baseline_20260929_v1/run_depth_audit_worker.py'

for episode in hard_0_e1_0001 hard_0_e1_0002; do
  output="${base}/${episode}"
  mkdir -p "$output"
  STEREO_RIG_VARIANT=frozen STEREO_CAPTURE_STRIDE=2 PYTHONPATH="$repo:/home/an/Documents/ChatGPT/球型机器人导航/experiments/stereo_vertical_baseline_20260929_v1" \
    /home/an/miniconda3/envs/ball_wm/bin/python "$worker" \
      --manifest "$repo/configs/benchmark_v1/hard_0_dev_manifest.json" \
      --split dev --episode-id "$episode" --policy goal_greedy \
      --scene-root /home/an/project/NavDP/assets/scenes \
      --data-root "$repo/data" --run-root "$output" \
      --lighting-profile fixed_camera_fill_v1 --headless --record-video \
      > "${base}/${episode}.log" 2>&1
done
