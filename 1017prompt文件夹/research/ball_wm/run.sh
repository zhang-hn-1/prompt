#!/bin/bash
set -e

CONDA_ENV="ball_wm"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

activate_env() {
    source "$(conda info --base)/etc/profile.d/conda.sh"
    conda activate "$CONDA_ENV"
    export OMNI_KIT_ACCEPT_EULA=YES
}

run_sim() {
    activate_env
    cd "$SCRIPT_DIR"
    python scripts/sim_env.py --headless --num_steps "${1:-500}"
}

run_record() {
    activate_env
    cd "$SCRIPT_DIR"
    python scripts/sim_env.py --headless --record --num_steps "${1:-500}" --video_fps "${2:-25}"
}

run_record_window() {
    activate_env
    cd "$SCRIPT_DIR"
    python scripts/sim_env.py --record --num_steps "${1:-500}" --video_fps "${2:-25}"
}

run_collect() {
    activate_env
    cd "$SCRIPT_DIR"
    python scripts/collect_data.py \
        --headless \
        --num_episodes "${1:-50}" \
        --episode_len "${2:-300}"
}

case "${1:-sim}" in
    sim)     run_sim "${2}" ;;
    record)  run_record "${2}" "${3}" ;;
    record_window) run_record_window "${2}" "${3}" ;;
    collect) run_collect "${2}" "${3}" "${4}" ;;
    *)
        echo "Usage: ./run.sh [sim|record|record_window|collect] [args...]"
        echo "  sim     [num_steps]                   - run simulation"
        echo "  record  [num_steps] [fps]             - run headless and save RGB videos"
        echo "  record_window [num_steps] [fps]       - save RGB videos without --no-window"
        echo "  collect [episodes] [len]              - collect data"
        ;;
esac
