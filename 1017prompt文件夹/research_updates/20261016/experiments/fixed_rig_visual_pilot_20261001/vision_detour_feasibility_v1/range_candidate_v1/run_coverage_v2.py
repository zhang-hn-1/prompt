"""Single follow-up: change only near-obstacle training coverage, keep V1 model fixed."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import torch

from range_pilot import (HERE, OBSTACLE, SEQUENCES, e1_gate, evaluate, load_sequence,
                         train_one_model)


EXTRA = HERE.parents[1] / "illuminated_replay_v1"
FIT = {
    "E0_direct": SEQUENCES["E0_train"],
    "E1_direct": SEQUENCES["E1_historical_test"],
    "E2_direct": SEQUENCES["E2_train"],
    "hard_0_E1_0001": EXTRA / "hard_0_e1_0001/depth_truth",
    "hard_0_E1_0002": EXTRA / "hard_0_e1_0002/depth_truth",
}
TEST = SEQUENCES["hard_0_E1_0003_historical_test"]
OUTPUT = HERE / "coverage_v2"


def main() -> None:
    torch.set_num_threads(2)
    OUTPUT.mkdir(exist_ok=True)
    data = {name: load_sequence(folder) for name, folder in FIT.items()}
    training = {key: np.concatenate([entry[key] for entry in data.values()])
                for key in ("rgb", "labels", "depth", "central", "times", "eye_ids")}
    training_near = int(((training["labels"] == OBSTACLE) & training["central"]
                         & np.isfinite(training["depth"]) & (training["depth"] <= 2.0)).sum())
    model, losses = train_one_model(training, torch.device("cpu"))
    held = load_sequence(TEST)
    metrics = evaluate(model, held, torch.device("cpu"))
    gate = e1_gate(metrics)
    gate["rejection_reasons"] = [reason.replace("E1", "hard_0 E1 0003")
                                 for reason in gate["rejection_reasons"]]
    result = {
        "protocol": str(HERE / "coverage_v2_protocol.md"),
        "fit_complete_sequences": {name: entry["complete_frames"] for name, entry in data.items()},
        "fit_near_obstacle_central_pixels": training_near,
        "unchanged_v1_architecture_seed_loss_epochs_and_thresholds": True,
        "training_losses": losses,
        "historical_hard_0_E1_0003_test": metrics,
        "read_only_gate": gate,
    }
    torch.save(model.cpu().state_dict(), OUTPUT / "range_model.pt")
    (OUTPUT / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
