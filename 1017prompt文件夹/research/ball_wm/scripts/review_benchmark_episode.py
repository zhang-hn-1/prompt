#!/usr/bin/env python3
"""Record the project owner's behavioral review of one audit bundle."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from benchmark.artifacts import (  # noqa: E402
    ALLOWED_BEHAVIOR_LABELS,
    ALLOWED_BEHAVIOR_STATUSES,
    review_episode_bundle,
)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode-dir", type=Path, required=True)
    parser.add_argument(
        "--status", required=True, choices=sorted(ALLOWED_BEHAVIOR_STATUSES)
    )
    parser.add_argument("--reviewer", required=True)
    parser.add_argument(
        "--labels",
        nargs="*",
        default=(),
        choices=sorted(ALLOWED_BEHAVIOR_LABELS),
    )
    parser.add_argument("--notes", default="")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        result = review_episode_bundle(
            args.episode_dir,
            status=args.status,
            reviewer=args.reviewer,
            labels=args.labels,
            notes=args.notes,
        )
    except (OSError, TypeError, ValueError, RuntimeError) as error:
        print(f"Behavior review failed: {error}", file=sys.stderr)
        return 2
    print(f"Behavior review saved: {result}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
