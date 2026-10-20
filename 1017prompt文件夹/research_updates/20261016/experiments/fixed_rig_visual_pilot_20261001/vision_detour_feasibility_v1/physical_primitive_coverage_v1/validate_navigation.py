"""Independently verify scored goal entry, contacts, and exact escape prefix."""

import json
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
NAV = HERE / "navigation_probe"


def main():
    difference = {}
    with np.load(HERE / "reverse_positive/physics.npz") as reference, np.load(NAV / "physics.npz") as candidate:
        for key in ("position_world_xy", "root_quaternion_world_wxyz", "linear_velocity_world_m_s",
                    "angular_velocity_world_rad_s", "joint_position_rad", "joint_velocity_rad_s",
                    "smoothed_target", "applied_action", "raw_command_target",
                    "simulation_time_s", "physics_step_index"):
            current = candidate[key][:len(reference[key])]
            difference[key] = float(np.max(np.abs(current.astype(float) - reference[key].astype(float))))
    with np.load(HERE / "reverse_positive/requests.npz") as reference, np.load(NAV / "episode_hard_0_e1_0004/actions.npz") as candidate:
        difference["request_blocks"] = float(np.max(np.abs(candidate["action_blocks"][:60] - reference["action_blocks"])))
    episode_folder = NAV / "episode_hard_0_e1_0004"
    metrics = json.loads((episode_folder / "metrics.json").read_text())
    contacts = json.loads((episode_folder / "collision_audit.json").read_text())["samples"]
    manifest = json.loads((episode_folder / "manifest.json").read_text())
    episode = next(e for e in manifest["episodes"] if e["episode_id"] == "hard_0_e1_0004")
    goal = np.asarray(episode["goal_position_world_m"][:2])
    with np.load(episode_folder / "states.npz") as state:
        distance = float(np.linalg.norm(state["position_world_xy"][-1] - goal))
        initial = state["position_world_xy"][0].tolist()
        final_speed = float(np.linalg.norm(state["linear_velocity_world_m_s"][-1, :2]))
    valid_contacts = all(s["available"] and not s["contact"] for s in contacts)
    passed = (all(d <= 1e-6 for d in difference.values()) and metrics["success"] == 1
              and metrics["collision"] == 0 and metrics["elapsed_s"] <= 35
              and metrics["episode_budget_s"] == 35 and distance <= 0.2 and valid_contacts)
    output = {"passed": passed, "prefix_max_abs_difference": difference,
              "goal_distance_m_independent": distance, "goal_world_xy": goal.tolist(),
              "initial_world_xy": initial, "all_scored_contacts_available_and_false": valid_contacts,
              "contact_samples": len(contacts), "elapsed_s": metrics["elapsed_s"],
              "terminal_xy_speed_m_s": final_speed,
              "goal_entry_success_does_not_certify_stop_hold": True}
    (NAV / "validation.json").write_text(json.dumps(output, indent=2) + "\n")
    print(json.dumps(output, indent=2))
    if not passed:
        raise SystemExit("candidate validation failed")


if __name__ == "__main__":
    main()
