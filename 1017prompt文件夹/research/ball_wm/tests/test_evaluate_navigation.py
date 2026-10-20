"""Isaac-free behavior tests for closed-loop navigation evaluation helpers."""

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch
from PIL import Image

from training.action_protocol import ActionProtocol

try:
    import training.evaluate_navigation as navigation
except ModuleNotFoundError:
    navigation = None

try:
    from training.evaluate_navigation import (
        NavigationContext,
        build_model_context,
        select_navigation_contexts,
    )
except ModuleNotFoundError:
    NavigationContext = None
    build_model_context = None
    select_navigation_contexts = None

try:
    from training.evaluate_navigation import _clip_action_block, collision_from_clearance
except ImportError:
    _clip_action_block = None
    collision_from_clearance = None

try:
    from training.evaluate_navigation import (
        RuntimeObservation,
        _observation_from_state,
    )
except ImportError:
    RuntimeObservation = None
    _observation_from_state = None

try:
    from training.evaluate_navigation import _run_closed_loop_episode, advance_model_context
except ImportError:
    _run_closed_loop_episode = None
    advance_model_context = None

try:
    from training.pose_utils import pointgoal_in_base
except ImportError:
    pointgoal_in_base = None

try:
    from training.evaluate_navigation import _recorded_goal_info, sample_random_action_block
except ImportError:
    _recorded_goal_info = None
    sample_random_action_block = None

try:
    from training.evaluate_navigation import _recorded_context_sample
except ImportError:
    _recorded_context_sample = None

try:
    from training.evaluate_navigation import _json_compatible
except ImportError:
    _json_compatible = None

try:
    from training.evaluate_navigation import evaluation_exit_code
except ImportError:
    evaluation_exit_code = None

try:
    from training.evaluate_navigation import require_checkpoint_protocol
except ImportError:
    require_checkpoint_protocol = None

try:
    from training.evaluate_navigation import load_context_manifest, write_context_manifest
except ImportError:
    load_context_manifest = None
    write_context_manifest = None

try:
    from training.evaluate_navigation import resolve_navigation_contexts
except ImportError:
    resolve_navigation_contexts = None


def _aux30_protocol() -> ActionProtocol:
    return ActionProtocol(
        protocol_id="ball_action_v2_aux30",
        minimum=(-2.0, -0.5235987756),
        maximum=(2.0, 0.5235987756),
        controls_per_block=5,
    )


def _write_episode(root: Path, name: str, scene: str, frames: int = 12) -> Path:
    episode = root / scene / name
    episode.mkdir(parents=True)
    np.savez_compressed(episode / "states.npz", stereo_frame_id=np.arange(frames, dtype=np.int64))
    (episode / "metadata.json").write_text(
        json.dumps({"schema_version": "1.2.0", "scene": f"/assets/{scene}/scene.usd"})
    )
    return episode


def _write_strict_episode(
    root: Path,
    name: str,
    scene: str,
    position_step_m: float,
    initially_colliding: bool = False,
    initial_clearance_m: float = 1.0,
) -> Path:
    """Create a raw episode whose valid PointGoal distance is hand-controlled."""

    frames = 60
    state_count = frames * 5
    episode = root / scene / name
    episode.mkdir(parents=True)
    positions = np.zeros((state_count, 3), dtype=np.float32)
    positions[:, 0] = np.arange(state_count, dtype=np.float32) * position_step_m
    np.savez_compressed(
        episode / "states.npz",
        stereo_frame_id=np.arange(frames, dtype=np.int64),
        pos=positions,
        collision=np.full(frames, initially_colliding, dtype=bool),
        minimum_obstacle_distance=np.full(frames, initial_clearance_m, dtype=np.float32),
    )
    (episode / "metadata.json").write_text(
        json.dumps({"schema_version": "1.2.0", "scene": f"/assets/{scene}/scene.usd"})
    )
    return episode


@pytest.fixture()
def context_dataset(tmp_path: Path):
    episodes = [
        _write_episode(tmp_path, "ep0000", "scene_easy_8"),
        _write_episode(tmp_path, "ep0001", "scene_easy_8"),
        _write_episode(tmp_path, "ep0000", "scene_easy_1"),
    ]
    return SimpleNamespace(
        samples=[(0, 0), (0, 1), (1, 0), (1, 1), (2, 0)],
        sample_scenes=["easy_8", "easy_8", "easy_8", "easy_8", "easy_1"],
        episode_paths=episodes,
        config=SimpleNamespace(history_frames=5),
    )


@pytest.fixture()
def strict_context_dataset(tmp_path: Path):
    """One long/safe, one short, and one initially colliding raw episode."""

    episodes = [
        _write_strict_episode(tmp_path, "ep0000", "scene_easy_8", position_step_m=0.01),
        _write_strict_episode(tmp_path, "ep0001", "scene_easy_8", position_step_m=0.001),
        _write_strict_episode(
            tmp_path,
            "ep0002",
            "scene_easy_8",
            position_step_m=0.01,
            initially_colliding=True,
        ),
    ]
    return SimpleNamespace(
        samples=[],
        sample_scenes=[],
        episode_paths=episodes,
        config=SimpleNamespace(history_frames=5),
    )


@pytest.fixture()
def model_sample() -> dict[str, torch.Tensor]:
    frames = 8
    return {
        "pixels_left": torch.arange(frames * 3 * 2 * 2, dtype=torch.float32).reshape(frames, 3, 2, 2),
        "pixels_right": torch.arange(frames * 3 * 2 * 2, dtype=torch.float32).reshape(frames, 3, 2, 2),
        "action": torch.ones(frames, 10, dtype=torch.float32),
        "proprio": torch.arange(frames * 17, dtype=torch.float32).reshape(frames, 17),
    }


def test_context_selection_uses_one_deterministic_context_per_episode(context_dataset):
    first = select_navigation_contexts(context_dataset, "scene_easy_8", 2, 5, 7)
    second = select_navigation_contexts(context_dataset, "scene_easy_8", 2, 5, 7)

    assert first == second
    assert {context.episode_index for context in first} == {0, 1}
    assert all(context.goal_frame == context.context_last_frame + 5 for context in first)
    assert all(context.start_state_index == 5 * (context.context_last_frame + 1) - 1 for context in first)
    assert all(context.goal_state_index == 5 * (context.goal_frame + 1) - 1 for context in first)


def test_context_selection_rejects_request_larger_than_available_episodes(context_dataset):
    with pytest.raises(ValueError, match="available"):
        select_navigation_contexts(context_dataset, "scene_easy_8", 3, 5, 7)


def test_context_selection_does_not_depend_on_model_rollout_sample_truncation(context_dataset):
    """Catches checkpoint-specific rollout horizons selecting different episodes."""

    truncated = SimpleNamespace(
        samples=[(0, 0), (1, 0), (2, 0)],
        sample_scenes=context_dataset.sample_scenes,
        episode_paths=context_dataset.episode_paths,
        config=SimpleNamespace(history_frames=5),
    )

    full_contexts = select_navigation_contexts(context_dataset, "scene_easy_8", 2, 5, 7)
    truncated_contexts = select_navigation_contexts(truncated, "scene_easy_8", 2, 5, 7)

    assert full_contexts == truncated_contexts


def test_strict_context_selection_excludes_short_and_initially_colliding_episodes(strict_context_dataset):
    """Catches a protocol that lets trivial goals or terminal starts into the suite."""

    contexts = select_navigation_contexts(
        strict_context_dataset,
        "scene_easy_8",
        0,
        20,
        3072,
        min_initial_goal_distance_m=0.5,
        require_initial_clearance=True,
    )

    assert len(contexts) == 1
    assert contexts[0].episode_index == 0
    assert contexts[0].initial_distance_m == pytest.approx(1.0)
    assert contexts[0].initial_clearance_m == pytest.approx(1.0)


def test_strict_context_selection_applies_minimum_initial_clearance(tmp_path: Path):
    """A positive clearance flag must support a meaningful safety margin."""

    episodes = [
        _write_strict_episode(
            tmp_path,
            "ep0000",
            "scene_easy_8",
            position_step_m=0.01,
            initial_clearance_m=0.4,
        ),
        _write_strict_episode(
            tmp_path,
            "ep0001",
            "scene_easy_8",
            position_step_m=0.01,
            initial_clearance_m=0.8,
        ),
    ]
    dataset = SimpleNamespace(
        samples=[],
        sample_scenes=[],
        episode_paths=episodes,
        config=SimpleNamespace(history_frames=5),
    )

    contexts = select_navigation_contexts(
        dataset,
        "scene_easy_8",
        0,
        20,
        3072,
        min_initial_goal_distance_m=0.5,
        require_initial_clearance=True,
        min_initial_clearance_m=0.5,
    )

    assert [context.episode_index for context in contexts] == [1]
    assert contexts[0].initial_clearance_m == pytest.approx(0.8)


def test_strict_context_selection_returns_at_most_one_context_per_episode(strict_context_dataset):
    """Catches a suite that inflates its sample count with correlated starts."""

    contexts = select_navigation_contexts(
        strict_context_dataset,
        "scene_easy_8",
        0,
        20,
        3072,
        min_initial_goal_distance_m=0.5,
        require_initial_clearance=True,
    )

    assert len({context.episode_index for context in contexts}) == len(contexts)


def test_context_manifest_round_trip_preserves_strict_contexts(strict_context_dataset, tmp_path: Path):
    """Catches policy cases silently resampling instead of consuming shared contexts."""

    contexts = select_navigation_contexts(
        strict_context_dataset,
        "scene_easy_8",
        0,
        20,
        3072,
        min_initial_goal_distance_m=0.5,
        require_initial_clearance=True,
    )
    manifest_path = tmp_path / "contexts.json"

    payload = write_context_manifest(
        manifest_path,
        tmp_path,
        5,
        20,
        0.5,
        True,
        3072,
        contexts,
    )
    loaded = load_context_manifest(manifest_path, tmp_path, strict_context_dataset, "scene_easy_8")

    assert payload["schema_version"] == "pointgoal_context_manifest_v1"
    assert payload["num_contexts"] == 1
    assert loaded == contexts


def test_context_manifest_rejects_changed_goal_frame(strict_context_dataset, tmp_path: Path):
    """Catches a manifest whose target no longer agrees with state-frame alignment."""

    contexts = select_navigation_contexts(
        strict_context_dataset,
        "scene_easy_8",
        0,
        20,
        3072,
        min_initial_goal_distance_m=0.5,
        require_initial_clearance=True,
    )
    manifest_path = tmp_path / "changed_goal.json"
    payload = write_context_manifest(manifest_path, tmp_path, 5, 20, 0.5, True, 3072, contexts)
    payload["contexts"][0]["goal_frame"] += 1
    manifest_path.write_text(json.dumps(payload))

    with pytest.raises(ValueError, match="goal_frame"):
        load_context_manifest(manifest_path, tmp_path, strict_context_dataset, "scene_easy_8")


def test_context_manifest_rejects_duplicate_episode(strict_context_dataset, tmp_path: Path):
    """Catches a protocol that inflates its sample count with duplicate episodes."""

    contexts = select_navigation_contexts(
        strict_context_dataset,
        "scene_easy_8",
        0,
        20,
        3072,
        min_initial_goal_distance_m=0.5,
        require_initial_clearance=True,
    )
    manifest_path = tmp_path / "duplicate.json"
    payload = write_context_manifest(manifest_path, tmp_path, 5, 20, 0.5, True, 3072, contexts)
    payload["contexts"].append(dict(payload["contexts"][0]))
    payload["num_contexts"] = 2
    manifest_path.write_text(json.dumps(payload))

    with pytest.raises(ValueError, match="duplicate episode_index"):
        load_context_manifest(manifest_path, tmp_path, strict_context_dataset, "scene_easy_8")


def test_context_manifest_overrides_seeded_resampling(strict_context_dataset, tmp_path: Path):
    """Catches one policy case changing its test set through a different seed."""

    contexts = select_navigation_contexts(
        strict_context_dataset,
        "scene_easy_8",
        0,
        20,
        3072,
        min_initial_goal_distance_m=0.5,
        require_initial_clearance=True,
    )
    manifest_path = tmp_path / "shared.json"
    write_context_manifest(manifest_path, tmp_path, 5, 20, 0.5, True, 3072, contexts)

    resolved = resolve_navigation_contexts(
        strict_context_dataset,
        "scene_easy_8",
        0,
        20,
        1,
        manifest_path,
        0.5,
        True,
    )

    assert resolved == contexts


def test_model_context_keeps_history_and_zeros_the_replaceable_action(model_sample):
    context = build_model_context(model_sample, history_size=5)

    assert set(context) == {"pixels_left", "pixels_right", "action", "proprio"}
    assert context["pixels_left"].shape == (1, 5, 3, 2, 2)
    assert context["proprio"].shape == (1, 5, 17)
    assert torch.equal(context["action"][0, 3], torch.ones(10))
    assert torch.count_nonzero(context["action"][0, -1]) == 0
    assert torch.count_nonzero(model_sample["action"][-1]) == 10


def test_action_block_is_clipped_to_the_physical_joint_bounds():
    block = np.tile(np.asarray([99.0, -99.0], dtype=np.float32), 5)

    clipped = _clip_action_block(block, _aux30_protocol())

    assert clipped.shape == (5, 2)
    assert np.all(clipped[:, 0] == 2.0)
    assert np.all(clipped[:, 1] == pytest.approx(-0.5235987756))


def test_clearance_at_or_below_zero_is_a_collision():
    assert collision_from_clearance(-1e-4) is True
    assert collision_from_clearance(0.0) is True
    assert collision_from_clearance(1e-4) is False
    assert collision_from_clearance(np.inf) is False


def test_cem_planner_protocol_defaults_to_odometry_and_gates_recorded_target_rgb():
    """Catches formal navigation silently consuming a future recorded target image."""

    assert navigation is not None
    resolve = getattr(navigation, "resolve_planner_goal_source", None)
    assert resolve is not None, "planner goal-source protocol guard is not available"

    assert resolve(
        policy="cem",
        planner_mode="pose_bev",
        pose_head_enabled=True,
        allow_target_image_diagnostic=False,
    ) == "wheel_odometry_pointgoal"
    with pytest.raises(ValueError, match="allow_target_image_diagnostic"):
        resolve(
            policy="cem",
            planner_mode="latent",
            pose_head_enabled=False,
            allow_target_image_diagnostic=False,
        )
    assert resolve(
        policy="cem",
        planner_mode="latent",
        pose_head_enabled=False,
        allow_target_image_diagnostic=True,
    ) == "recorded_target_rgb_diagnostic"
    with pytest.raises(ValueError, match="pose head"):
        resolve(
            policy="cem",
            planner_mode="pose_bev",
            pose_head_enabled=False,
            allow_target_image_diagnostic=False,
        )


def test_runtime_observation_uses_current_state_and_marks_collision():
    """Catches a runtime observation that drops simulator pose or clearance."""

    rgb = np.full((2, 2, 3), 128, dtype=np.uint8)
    observation = _observation_from_state(
        left_rgb=rgb,
        right_rgb=rgb,
        position=np.asarray([1.5, -2.0, 0.45]),
        quaternion_wxyz=np.asarray([1.0, 0.0, 0.0, 0.0]),
        linear_velocity=np.asarray([0.1, 0.2, 0.3]),
        angular_velocity=np.asarray([0.4, 0.5, 0.6]),
        joint_positions=np.asarray([0.7, 0.8, 0.9]),
        joint_velocities=np.asarray([1.0, 1.1, 1.2]),
        previous_action=np.asarray([1.3, 1.4]),
        clearance_m=-0.01,
        image_size=(2, 2),
    )

    assert observation.pixels_left.shape == (3, 2, 2)
    assert observation.pixels_right.shape == (3, 2, 2)
    assert observation.proprio.shape == (17,)
    np.testing.assert_allclose(observation.position_xy, [1.5, -2.0])
    assert observation.yaw == pytest.approx(0.0)
    assert observation.collision is True
    np.testing.assert_allclose(observation.trajectory_xy, [[1.5, -2.0]])


def test_advancing_context_retains_executed_action_and_replaces_observation(model_sample):
    """Catches an MPC rollout that drops the action just executed in Isaac."""

    context = build_model_context(model_sample, history_size=5)
    observation = RuntimeObservation(
        pixels_left=torch.full((3, 2, 2), 7.0),
        pixels_right=torch.full((3, 2, 2), 8.0),
        proprio=torch.full((17,), 9.0),
        position_xy=np.asarray([0.0, 0.0]),
        yaw=0.0,
        clearance_m=1.0,
        collision=False,
        trajectory_xy=np.asarray([[0.0, 0.0]]),
    )
    executed = np.arange(10, dtype=np.float32)

    advanced = advance_model_context(context, executed, observation)

    assert advanced["pixels_left"].shape == (1, 5, 3, 2, 2)
    assert torch.equal(advanced["pixels_left"][0, -1], observation.pixels_left)
    assert torch.equal(advanced["proprio"][0, -1], observation.proprio)
    np.testing.assert_allclose(advanced["action"][0, -2].numpy(), executed)
    assert torch.count_nonzero(advanced["action"][0, -1]) == 0


class _SuccessAfterOneActionRuntime:
    """Minimal external-device substitute; result assertions target the episode loop."""

    def execute_action_block(self, _action_block: np.ndarray) -> RuntimeObservation:
        return RuntimeObservation(
            pixels_left=torch.zeros(3, 2, 2),
            pixels_right=torch.zeros(3, 2, 2),
            proprio=torch.zeros(17),
            position_xy=np.asarray([0.9, 0.0]),
            yaw=0.0,
            clearance_m=1.0,
            collision=False,
            trajectory_xy=np.asarray([[0.0, 0.0], [0.9, 0.0]]),
        )


def test_closed_loop_selector_receives_current_base_goal(model_sample):
    """Catches a PointGoal evaluator that reuses recorded target imagery instead of live odometry."""

    assert pointgoal_in_base is not None
    received: list[np.ndarray] = []
    initial_observation = RuntimeObservation(
        pixels_left=torch.zeros(3, 2, 2),
        pixels_right=torch.zeros(3, 2, 2),
        proprio=torch.zeros(17),
        position_xy=np.asarray([0.0, 0.0]),
        yaw=np.pi / 2.0,
        clearance_m=1.0,
        collision=False,
        trajectory_xy=np.asarray([[0.0, 0.0]]),
    )

    result = _run_closed_loop_episode(
        runtime=_SuccessAfterOneActionRuntime(),
        context=build_model_context(model_sample, history_size=5),
        initial_position_xy=initial_observation.position_xy,
        initial_observation=initial_observation,
        goal_xy=np.asarray([0.0, 1.0]),
        action_selector=lambda _ctx, obs, goal: received.append(pointgoal_in_base(obs.position_xy, obs.yaw, goal))
        or np.zeros(10, dtype=np.float32),
        max_mpc_steps=1,
        goal_tolerance_m=0.01,
        reference_length_m=1.0,
    )

    np.testing.assert_allclose(received, [[1.0, 0.0]], atol=1e-6)
    assert result["steps"] == 1


def test_episode_loop_stops_on_success_before_timeout(model_sample):
    """Catches a closed-loop evaluator that ignores a reached goal until timeout."""

    result = _run_closed_loop_episode(
        runtime=_SuccessAfterOneActionRuntime(),
        context=build_model_context(model_sample, history_size=5),
        initial_position_xy=np.asarray([0.0, 0.0]),
        goal_xy=np.asarray([1.0, 0.0]),
        action_selector=lambda _context: np.zeros(10, dtype=np.float32),
        max_mpc_steps=50,
        goal_tolerance_m=0.2,
        reference_length_m=1.0,
    )

    assert result["termination_reason"] == "success"
    assert result["steps"] == 1
    assert result["final_distance_m"] == pytest.approx(0.1)
    assert result["path_length_m"] == pytest.approx(0.9)


def test_diagnostic_action_trace_pairs_plan_costs_with_executed_outcome(model_sample):
    """Catches a diagnostic trace that cannot link a CEM decision to real progress."""

    assert navigation is not None
    assert hasattr(navigation, "PlannerAction"), "planner diagnostics need an action/result boundary"
    decision = navigation.PlannerAction(
        action_block=np.zeros(10, dtype=np.float32),
        diagnostics={
            "total_cost": 1.5,
            "latent_goal_cost": 1.25,
            "bev_safety_cost": 1.0,
            "elite_cost_min": 1.5,
            "elite_cost_mean": 1.7,
            "elite_cost_std": 0.2,
        },
        goal_relative_xy=np.asarray([1.0, 0.0], dtype=np.float32),
        predicted_relative_trajectory=np.asarray(
            [[0.2, 0.0, 0.0, 1.0], [0.4, 0.0, 0.0, 1.0]], dtype=np.float32
        ),
    )

    result = _run_closed_loop_episode(
        runtime=_SuccessAfterOneActionRuntime(),
        context=build_model_context(model_sample, history_size=5),
        initial_position_xy=np.asarray([0.0, 0.0]),
        goal_xy=np.asarray([1.0, 0.0]),
        action_selector=lambda _context: decision,
        max_mpc_steps=50,
        goal_tolerance_m=0.2,
        reference_length_m=1.0,
        collect_planner_trace=True,
    )

    assert len(result["planner_trace"]) == 1
    trace = result["planner_trace"][0]
    assert trace["step"] == 1
    assert trace["selected_action_block"] == [0.0] * 10
    assert trace["total_cost"] == pytest.approx(1.5)
    assert trace["latent_goal_cost"] == pytest.approx(1.25)
    assert trace["bev_safety_cost"] == pytest.approx(1.0)
    assert trace["elite_cost_min"] == pytest.approx(1.5)
    assert trace["elite_cost_mean"] == pytest.approx(1.7)
    assert trace["elite_cost_std"] == pytest.approx(0.2)
    assert trace["goal_relative_xy"] == [1.0, 0.0]
    np.testing.assert_allclose(
        trace["predicted_relative_trajectory"],
        [[0.2, 0.0, 0.0, 1.0], [0.4, 0.0, 0.0, 1.0]],
        atol=1e-6,
    )
    np.testing.assert_allclose(trace["actual_position_xy"], [0.9, 0.0])
    assert trace["actual_distance_m"] == pytest.approx(0.1)
    assert trace["distance_delta_m"] == pytest.approx(-0.9)
    assert trace["actual_clearance_m"] == pytest.approx(1.0)
    assert trace["actual_collision"] is False


def test_diagnostic_trajectory_renderer_writes_a_readable_png(tmp_path: Path):
    """Catches a diagnostics command that emits trace JSON but no inspectable trajectory figure."""

    assert navigation is not None
    assert hasattr(navigation, "render_diagnostic_trajectory_plot"), "diagnostic trajectories need a PNG renderer"
    output_path = navigation.render_diagnostic_trajectory_plot(
        tmp_path / "diagnostic.png",
        [
            {
                "trajectory_xy": [[0.0, 0.0], [0.5, 0.0]],
                "goal_xy": [1.0, 0.0],
                "termination_reason": "timeout",
            }
        ],
        title="diagnostic fixture",
    )

    assert output_path == tmp_path / "diagnostic.png"
    with Image.open(output_path) as image:
        assert image.mode == "RGB"
        assert image.width >= 640
        assert image.height >= 480


def test_recorded_goal_embedding_input_uses_only_target_frame(tmp_path: Path):
    """Catches accidental use of target-frame future actions or images in CEM goals."""

    episode = tmp_path / "scene_easy_8" / "ep0000"
    episode.mkdir(parents=True)
    state_count = 30
    np.savez_compressed(
        episode / "states.npz",
        stereo_frame_id=np.arange(6, dtype=np.int64),
        pos=np.zeros((state_count, 3), dtype=np.float32),
        quat=np.tile(np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32), (state_count, 1)),
        lin_vel=np.zeros((state_count, 3), dtype=np.float32),
        ang_vel=np.zeros((state_count, 3), dtype=np.float32),
        joint_pos=np.zeros((state_count, 3), dtype=np.float32),
        joint_vel=np.zeros((state_count, 3), dtype=np.float32),
        applied_action=np.full((state_count, 2), 7.0, dtype=np.float32),
    )
    (episode / "metadata.json").write_text(json.dumps({"schema_version": "1.2.0"}))
    for eye in ("left", "right"):
        for frame in range(6):
            Image.fromarray(np.full((2, 2, 3), frame * 255, dtype=np.uint8)).save(
                episode / f"{eye}_{frame:06d}.png"
            )
    dataset = SimpleNamespace(
        episode_paths=[episode],
        config=SimpleNamespace(image_size=(2, 2), normalize=True, history_frames=5, action_block=5),
    )

    goal = _recorded_goal_info(dataset, episode_index=0, goal_frame=0)

    assert goal["pixels_left"].shape == (1, 1, 3, 2, 2)
    assert goal["pixels_right"].shape == (1, 1, 3, 2, 2)
    assert goal["proprio"].shape == (1, 1, 17)
    assert torch.count_nonzero(goal["action"]) == 0

    context = _recorded_context_sample(dataset, episode_index=0, context_start=0)
    assert context["pixels_left"].shape == (5, 3, 2, 2)
    assert context["pixels_right"].shape == (5, 3, 2, 2)
    assert context["proprio"].shape == (5, 17)
    assert context["action"].shape == (5, 10)


def test_random_policy_action_blocks_are_seeded_and_within_joint_bounds():
    """Catches a random baseline whose actions are not comparable to CEM bounds."""

    first = sample_random_action_block(np.random.default_rng(3072), _aux30_protocol())
    second = sample_random_action_block(np.random.default_rng(3072), _aux30_protocol())

    np.testing.assert_allclose(first, second)
    assert first.shape == (10,)
    assert np.all(first[0::2] >= -2.0)
    assert np.all(first[0::2] <= 2.0)
    assert np.all(first[1::2] >= -0.5235987756)
    assert np.all(first[1::2] <= 0.5235987756)


def test_v2_evaluator_rejects_unlabelled_checkpoint_as_legacy():
    """An old checkpoint must not silently inherit the wider V2 steering range."""
    assert require_checkpoint_protocol is not None

    with pytest.raises(ValueError, match="ball_action_v1_aux16"):
        require_checkpoint_protocol({"model": {}}, "ball_action_v2_aux30")


def test_v2_evaluator_accepts_matching_checkpoint_protocol():
    checkpoint = {"model": {}, "action_protocol_id": "ball_action_v2_aux30"}

    assert (
        require_checkpoint_protocol(checkpoint, "ball_action_v2_aux30")
        == "ball_action_v2_aux30"
    )


def test_json_output_replaces_non_finite_diagnostics_with_null():
    """Catches `Infinity` in strict result JSON for maps with no obstacles."""

    payload = _json_compatible(
        {"clearance": np.inf, "distance": np.float32(0.25), "trajectory": np.asarray([[1.0, 2.0]])}
    )

    assert payload["clearance"] is None
    assert payload["distance"] == pytest.approx(0.25)
    assert payload["trajectory"] == [[1.0, 2.0]]
    json.dumps(payload, allow_nan=False)


def test_runtime_errors_make_the_cli_exit_nonzero():
    """Catches a CLI that serializes runtime errors but reports shell success."""

    assert evaluation_exit_code({"runtime_error_count": 0}) == 0
    assert evaluation_exit_code({"runtime_error_count": 1}) == 1
