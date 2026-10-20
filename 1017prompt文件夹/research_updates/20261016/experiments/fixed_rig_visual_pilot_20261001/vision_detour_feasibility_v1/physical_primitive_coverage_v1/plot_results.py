"""Physical local branches and full 35 s Oracle continuation comparison."""

import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np

from assess import PROJECT, occupied_cell_distances
from primitive_measurement import PRIMITIVES


HERE = Path(__file__).resolve().parent
NAV = HERE / "navigation_probe"


def map_background(ax, grid, origin, resolution):
    extent = (origin[0] - resolution / 2, origin[0] + (grid.shape[0] - 0.5) * resolution,
              origin[1] - resolution / 2, origin[1] + (grid.shape[1] - 0.5) * resolution)
    ax.imshow(grid.T, origin="lower", extent=extent,
              cmap=ListedColormap(["#fafafa", "#606060", "#d5d5d5"]),
              vmin=0, vmax=2, interpolation="nearest")
    ax.set_xlabel("world X (m)")
    ax.set_ylabel("world Y (m)")
    ax.set_aspect("equal")
    ax.grid(alpha=0.18)


def main():
    with np.load(PROJECT / "data/benchmark_v1/occupancy/cluttered_hard_0_bodyband.npz") as occ:
        grid = occ["grid"]
        origin = occ["global_origin_xy"]
        resolution = float(occ["resolution_m"])
    assessment = json.loads((HERE / "assessment.json").read_text())
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.3), constrained_layout=True)
    map_background(axes[0], grid, origin, resolution)
    colors = ["#1f77b4", "#9467bd", "#ff7f0e", "#17becf", "#a6761d", "#229b43", "#222222"]
    labels = {"forward_straight": "forward, u=0", "forward_negative": "forward, u=-30 deg",
              "forward_positive": "forward, u=+30 deg", "reverse_straight": "reverse, u=0",
              "reverse_negative": "reverse, u=-30 deg", "reverse_positive": "reverse, u=+30 deg (6 s clear)",
              "stop": "stop (no moving alternative)"}
    all_xy = []
    for (case, _), color in zip(PRIMITIVES.items(), colors):
        with np.load(HERE / case / "physics.npz") as state:
            xy = state["position_world_xy"]
            time = state["simulation_time_s"]
        all_xy.append(xy)
        axes[0].plot(xy[:, 0], xy[:, 1], color=color, lw=2, label=labels[case])
        centers, _ = occupied_cell_distances(xy, grid, origin, resolution)
        axes[1].plot(time, centers, color=color, lw=1.8, label=labels[case])
        contact = assessment["results"][case]["map_contact_geometry"]["first_contact"]
        if contact:
            axes[0].scatter(*contact["xy"], color=color, marker="x", s=70, lw=2, zorder=4)
            axes[1].scatter(contact["time_s"], contact["distance_to_occupied_cell_centers_m"],
                            color=color, marker="x", s=65, lw=2, zorder=4)
    envelope = np.concatenate(all_xy)
    axes[0].set_xlim(envelope[:, 0].min() - 0.8, envelope[:, 0].max() + 0.8)
    axes[0].set_ylim(envelope[:, 1].min() - 0.65, envelope[:, 1].max() + 0.75)
    axes[0].scatter(*all_xy[0][0], color="black", s=30, zorder=6)
    axes[0].set_title("Same initial state: actual primitive trajectories")
    axes[1].axhline(0.4, color="#777777", ls=":", label="0.40 m shell radius")
    axes[1].set_xlabel("simulation time (s)")
    axes[1].set_ylabel("distance to occupied cell centers (m)")
    axes[1].set_title("Measured contact events are marked with x")
    axes[1].grid(alpha=0.22)
    axes[1].legend(fontsize=7.6, loc="upper left")
    fig.savefig(HERE / "physical_primitive_comparison.png", dpi=170)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5.6), constrained_layout=True)
    map_background(axes[0], grid, origin, resolution)
    validation = json.loads((NAV / "validation.json").read_text())
    goal = np.asarray(validation["goal_world_xy"])
    with np.load(NAV / "episode_hard_0_e1_0004/states.npz") as state:
        actual = state["position_world_xy"]
        t = state["simulation_time_s"]
    with np.load(HERE / "b0_replay/states.npz") as state:
        b0 = state["position_world_xy"]
        bt = state["simulation_time_s"]
    with np.load(NAV / "reference_built_at_6s.npz") as route:
        path = route["position_world_xy"]
    axes[0].plot(path[:, 0], path[:, 1], "--", color="#437cbd", lw=1.5,
                 label="Oracle route built from current state at 6 s")
    axes[0].plot(actual[:, 0], actual[:, 1], color="#229b43", lw=2.3,
                 label="actual: goal entry at 28.4 s, no contact")
    axes[0].plot(b0[:, 0], b0[:, 1], color="#d8473e", lw=2.3,
                 label="B0: collision at 2.4 s")
    axes[0].scatter(*actual[0], color="black", s=40, label="same start", zorder=6)
    axes[0].scatter(*actual[60], color="#df8a15", s=48, label="6 s measured escape endpoint", zorder=6)
    axes[0].scatter(*goal, marker="*", color="#2860b5", s=140, label="goal", zorder=6)
    axes[0].scatter(*b0[-1], marker="x", color="#d8473e", s=70, lw=2, zorder=6)
    extent = np.vstack((actual, goal))
    axes[0].set_xlim(extent[:, 0].min() - 0.9, extent[:, 0].max() + 0.9)
    axes[0].set_ylim(extent[:, 1].min() - 0.8, extent[:, 1].max() + 0.9)
    axes[0].set_title("hard_0 E1 0004: privileged Oracle development probe")
    axes[0].legend(loc="lower left", fontsize=7.3, framealpha=0.95)
    axes[1].plot(t, np.linalg.norm(actual - goal, axis=1), color="#229b43", lw=2.2, label="Oracle continuation")
    axes[1].plot(bt, np.linalg.norm(b0 - goal, axis=1), color="#d8473e", lw=2.2, label="B0")
    axes[1].axvline(6, color="#df8a15", ls="--", lw=1.3, label="escape to route transition")
    axes[1].axhline(0.2, color="#437cbd", ls=":", label="formal goal tolerance (0.20 m)")
    axes[1].set_xlabel("simulation time (s)")
    axes[1].set_ylabel("actual distance to manifest goal (m)")
    axes[1].set_title("Original 35 s rule; actual goal entry, no stop-hold claim")
    axes[1].set_xlim(0, 35)
    axes[1].grid(alpha=0.22)
    axes[1].legend(fontsize=8)
    fig.savefig(HERE / "oracle_continuation_navigation_comparison.png", dpi=170)
    print(HERE / "oracle_continuation_navigation_comparison.png")


if __name__ == "__main__":
    main()
