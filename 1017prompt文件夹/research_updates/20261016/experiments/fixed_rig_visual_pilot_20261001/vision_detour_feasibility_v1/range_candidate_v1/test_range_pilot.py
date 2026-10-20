"""Inference must remain RGB-only; offline labels cannot enter the network input."""

import numpy as np
import pytest
import torch

from range_pilot import RangeNet, make_input, joint_near_obstacle, training_loss


def test_rgb_only_input_and_output_shapes() -> None:
    rgb = np.zeros((2, 120, 160, 3), dtype=np.uint8)
    x = make_input(rgb, np.array([0, 1]), torch.device("cpu"))
    assert x.shape == (2, 6, 120, 160)
    assert not torch.equal(x[0, 5], x[1, 5])
    assert RangeNet()(x).shape == (2, 4, 120, 160)
    with pytest.raises(ValueError):
        make_input(np.zeros((2, 120, 160, 4), dtype=np.uint8), np.array([0, 1]), torch.device("cpu"))


def test_frozen_joint_near_obstacle_gate() -> None:
    probability = np.array([0.49, 0.50, 0.9, 0.8])
    distance = np.array([1.0, 2.5, 2.51, 2.0])
    assert joint_near_obstacle(probability, distance).tolist() == [False, True, False, True]


def test_empty_obstacle_batch_has_finite_loss_and_gradient() -> None:
    output = torch.zeros((1, 4, 3, 3), requires_grad=True)
    labels = torch.zeros((1, 3, 3), dtype=torch.long)
    depth = torch.full((1, 3, 3), float("inf"))
    loss = training_loss(output, labels, depth, torch.nn.CrossEntropyLoss())
    assert torch.isfinite(loss)
    loss.backward()
    assert torch.isfinite(output.grad).all()


def test_all_known_depth_supervision_updates_ground_range_head() -> None:
    output = torch.zeros((1, 4, 3, 3), requires_grad=True)
    labels = torch.zeros((1, 3, 3), dtype=torch.long)
    depth = torch.full((1, 3, 3), 2.0)
    loss = training_loss(output, labels, depth, torch.nn.CrossEntropyLoss(),
                         depth_supervision="all_known")
    loss.backward()
    assert torch.isfinite(loss)
    assert torch.abs(output.grad[:, 3]).sum() > 0
