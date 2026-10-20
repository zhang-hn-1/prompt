"""Episode-directory discovery shared by training and dataset diagnostics."""

from __future__ import annotations

from pathlib import Path
import re
from typing import Sequence


def discover_episode_paths(
    roots: str | Path | Sequence[str | Path],
) -> list[Path]:
    """Return finalized episode directories under one or more data roots.

    Collection outputs may be flat (``root/ep0000``) or grouped by scene
    (``root/scene_easy_0/ep0000``).  Looking for paired ``states.npz`` and
    ``metadata.json`` files handles both layouts while excluding partial or
    unrelated directories.
    """

    root_values = [roots] if isinstance(roots, (str, Path)) else list(roots)
    episodes: set[Path] = set()
    for root_value in root_values:
        root = Path(root_value)
        if not root.is_dir():
            continue
        for states_path in root.rglob("states.npz"):
            episode = states_path.parent
            if (
                re.fullmatch(r"ep\d+", episode.name)
                and not any(part.endswith(".partial") for part in episode.parts)
                and (episode / "metadata.json").is_file()
            ):
                episodes.add(episode)
    return sorted(episodes)
