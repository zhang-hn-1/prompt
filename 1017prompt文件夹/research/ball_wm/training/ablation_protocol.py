"""Fixed scene splits and paired, budget-independent navigation prefixes."""
from copy import deepcopy
import math
import numpy as np
from .navigation_metrics import episode_termination, path_length


def scene_indices(sample_scenes, split):
    groups = [list(split[key]) for key in ('train', 'val', 'test')]
    flat = sum(groups, [])
    if any(not group for group in groups) or len(flat) != len(set(flat)):
        raise ValueError('scene split must be nonempty and disjoint')
    if set(flat) != set(sample_scenes):
        raise ValueError('scene split must cover exactly the dataset scenes')
    return tuple([i for i, scene in enumerate(sample_scenes) if scene in group] for group in groups)


def budget_episode(episode, budget, tolerance):
    """Truncate one longest-budget rollout; reject incomplete/error traces.

    Valid only for policies whose action selection is independent of the remaining
    evaluation budget. Collision precedes success, as in the runtime evaluator.
    """
    if budget < 1 or episode['termination_reason'] == 'runtime_error':
        raise ValueError('invalid budget or runtime error')
    steps = int(episode['steps'])
    trace = episode.get('planner_trace', [])
    points = episode['trajectory_xy']
    if len(trace) != steps or len(points) != steps + 1:
        raise ValueError('incomplete trajectory/trace')
    if steps < budget and episode['termination_reason'] == 'timeout':
        raise ValueError('source rollout did not run long enough')
    if [row['step'] for row in trace] != list(range(1, steps + 1)):
        raise ValueError('noncontiguous trace')
    result = deepcopy(episode)
    keep = min(steps, budget)
    if not keep:
        return result
    row = trace[keep - 1]
    distance = float(row['actual_distance_m'])
    reason = episode_termination(distance, row['actual_collision'], keep, budget, tolerance)
    if reason is None:
        raise ValueError('source termination inconsistent with trace')
    last_second = trace[max(0, keep - 10):keep]
    start_distance = (float(episode['initial_distance_m']) if keep <= 10
                      else float(trace[keep - 11]['actual_distance_m']))
    deltas = np.diff([start_distance] + [float(r['actual_distance_m']) for r in last_second])
    if not np.isfinite(deltas).all() or not math.isfinite(distance):
        raise ValueError('nonfinite distance trace')
    result.update(steps=keep, termination_reason=reason, collision=reason == 'collision',
                  trajectory_xy=points[:keep + 1], planner_trace=trace[:keep],
                  final_distance_m=distance, path_length_m=path_length(points[:keep + 1]),
                  min_clearance_m=min([float(episode['initial_clearance_m'])] +
                                      [float(r['actual_clearance_m']) for r in trace[:keep]]),
                  tail_progress_m=start_distance-distance,
                  tail_distance_increase_fraction=float(np.mean(deltas > 0)))
    return result
