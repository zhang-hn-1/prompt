"""Pure torch tests for candidate pose and pairwise ranking losses."""

import pytest
import torch

from training.candidate_losses import (
    candidate_goal_distances,
    candidate_pose_loss,
    candidate_ranking_accuracy,
    pairwise_candidate_ranking_loss,
)


def test_candidate_goal_distances_uses_final_xy_and_base_frame_goal():
    predicted = torch.zeros(1, 2, 5, 4)
    predicted[..., -1, :2] = torch.tensor([[[1.0, 0.0], [0.0, 2.0]]])
    goal = torch.tensor([[0.0, 0.0]])

    torch.testing.assert_close(candidate_goal_distances(predicted, goal), torch.tensor([[1.0, 2.0]]))


def test_pairwise_ranking_loss_penalizes_reversed_order():
    predicted = torch.tensor([[3.0, 2.0, 1.0]])
    target = torch.tensor([[1.0, 2.0, 3.0]])

    loss = pairwise_candidate_ranking_loss(predicted, target, margin_m=0.1)

    assert loss > 0
    assert candidate_ranking_accuracy(predicted, target, margin_m=0.1) == 0


def test_correct_order_has_zero_loss_and_collision_is_worse():
    predicted = torch.tensor([[1.0, 3.0, 2.0]])
    target = torch.tensor([[1.0, 3.0, 2.0]])
    collision = torch.tensor([[0, 1, 0]], dtype=torch.bool)

    assert pairwise_candidate_ranking_loss(predicted, target, collision, margin_m=0.05) == 0
    assert candidate_ranking_accuracy(predicted, target, collision, margin_m=0.05) == 1


def test_candidate_pose_loss_backpropagates_and_validates_shapes():
    predicted = torch.zeros(2, 8, 5, 4, requires_grad=True)
    target = torch.zeros_like(predicted)
    target[..., 3] = 1.0

    loss = candidate_pose_loss(predicted, target)
    loss.backward()

    assert predicted.grad is not None
    with pytest.raises(ValueError, match="shape"):
        candidate_pose_loss(torch.zeros(2, 8, 4), target)
