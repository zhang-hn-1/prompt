"""Draw the frozen candidate bank over map labels and executed XY tracks."""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap

from curve_labels import candidate_centerline
from run_label_audit import EPISODES, HERE, MAP_FILES, SIDES, yaw_from_quaternion


CASES = (
    ("E1", "E1_direct_fit", "e1", HERE.parent / "bilateral_depth_detour_e1/episode_golden_e1_0001/states.npz"),
    ("hard_0 0004", "hard_0_0004_test", "hard0", HERE.parent / "bilateral_depth_h4/episode_hard_0_e1_0004/states.npz"),
)


def main() -> None:
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.4), constrained_layout=True)
    line_colors = {"straight": "#165c97", "left": "#ae44a1", "right": "#df8313"}
    map_colors = ListedColormap(["#fafafa", "#555555", "#cccccc"])
    for ax, (title, episode_key, map_key, state_path) in zip(axes, CASES):
        frame_folder = EPISODES[episode_key][0]
        with np.load(sorted(frame_folder.glob("frame_*.npz"))[0]) as frame:
            start = frame["root_position_world_m"][:2]
            yaw = yaw_from_quaternion(frame["root_quaternion_world_wxyz"])
        with np.load(MAP_FILES[map_key]) as occ:
            grid = occ["grid"]
            origin = occ["global_origin_xy"]
            resolution = float(occ["resolution_m"])
        extent = (origin[0] - resolution / 2,
                  origin[0] + (grid.shape[0] - 0.5) * resolution,
                  origin[1] - resolution / 2,
                  origin[1] + (grid.shape[1] - 0.5) * resolution)
        ax.imshow(grid.T, origin="lower", extent=extent, cmap=map_colors,
                  vmin=0, vmax=2, interpolation="nearest")
        plotted = []
        for side in SIDES:
            curve = candidate_centerline(start, yaw, side)
            plotted.append(curve)
            ax.plot(curve[:, 0], curve[:, 1], lw=2.2,
                    color=line_colors[side], label=f"{side}: blocked at 0.60 m")
        with np.load(state_path) as state:
            track = state["position_world_xy"]
        plotted.append(track)
        label = "executed: success" if title == "E1" else "executed: collision"
        ax.plot(track[:, 0], track[:, 1], color="#32b65a", lw=2.4,
                label=label, zorder=4)
        ax.scatter(*start, color="black", s=45, zorder=5)
        ax.scatter(*track[-1], marker="x", color="#de322e", s=65,
                   linewidth=2.1, zorder=5)
        envelope = np.concatenate(plotted)
        ax.set_xlim(envelope[:, 0].min() - 0.8, envelope[:, 0].max() + 0.8)
        ax.set_ylim(envelope[:, 1].min() - 0.8, envelope[:, 1].max() + 0.8)
        ax.set_aspect("equal")
        ax.set_title(f"{title}: local candidate bank at start")
        ax.set_xlabel("world X (m)")
        ax.set_ylabel("world Y (m)")
        ax.grid(alpha=0.18)
        ax.legend(loc="lower left", fontsize=8, framealpha=0.92)
    output = HERE / "candidate_curve_audit.png"
    fig.savefig(output, dpi=160)
    print(output)


if __name__ == "__main__":
    main()
