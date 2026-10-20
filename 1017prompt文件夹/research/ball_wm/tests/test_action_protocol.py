from pathlib import Path
import sys

import numpy as np
import pytest
import torch


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from collection_schema import load_contract
from training.action_protocol import ActionProtocol, load_action_protocol


CONTRACT_PATH = Path(__file__).resolve().parents[1] / "configs" / "data_contract.yaml"


def aux30_protocol():
    return ActionProtocol.from_contract(load_contract(CONTRACT_PATH))


def test_action_protocol_expands_five_aux30_control_pairs():
    """Dropping or shortening a control pair would give the planner wrong block bounds."""
    protocol = aux30_protocol()

    bounds = protocol.block_bounds_numpy()

    assert bounds.shape == (10, 2)
    np.testing.assert_allclose(
        bounds,
        np.asarray([[-2.0, 2.0], [-0.5235987756, 0.5235987756]] * 5),
    )


def test_clip_block_enforces_each_component_bound_without_mutating_input():
    """Clipping every value as velocity would let auxiliary commands exceed 30 degrees."""
    protocol = aux30_protocol()
    source = np.asarray([3.0, 1.0] * 5, dtype=np.float64)

    clipped = protocol.clip_block(source)

    np.testing.assert_allclose(clipped, np.asarray([2.0, 0.5235987756] * 5))
    np.testing.assert_array_equal(source, np.asarray([3.0, 1.0] * 5))
    assert clipped.shape == (10,)
    assert clipped.dtype == np.float32


@pytest.mark.parametrize("invalid", [np.zeros(2), np.full(10, np.nan)])
def test_clip_block_rejects_wrong_shape_or_non_finite_values(invalid):
    """Malformed planner output must not reach the simulator as a partially valid command."""
    with pytest.raises(ValueError):
        aux30_protocol().clip_block(invalid)


def test_sample_block_is_reproducible_float32_and_within_component_bounds():
    """Sampling with the wrong layout could explore illegal steering commands."""
    protocol = aux30_protocol()

    first = protocol.sample_block(np.random.default_rng(3072))
    second = protocol.sample_block(np.random.default_rng(3072))
    bounds = protocol.block_bounds_numpy()

    np.testing.assert_array_equal(first, second)
    assert first.shape == (10,)
    assert first.dtype == np.float32
    assert np.all(first >= bounds[:, 0])
    assert np.all(first <= bounds[:, 1])


def test_sample_block_rejects_legacy_random_state():
    """Accepting multiple RNG APIs would make benchmark sampling ambiguous."""
    with pytest.raises(TypeError, match="Generator"):
        aux30_protocol().sample_block(np.random.RandomState(3072))


def test_torch_block_bounds_match_numpy_contract():
    """Planner tensors must use the same bounds and ordering as collection."""
    protocol = aux30_protocol()

    bounds = protocol.block_bounds_torch()

    assert isinstance(bounds, torch.Tensor)
    assert bounds.shape == (10, 2)
    assert bounds.dtype == torch.float32
    torch.testing.assert_close(bounds, torch.from_numpy(protocol.block_bounds_numpy()))


def test_shared_loader_reads_the_repository_contract():
    protocol = load_action_protocol(CONTRACT_PATH)

    assert protocol == aux30_protocol()
