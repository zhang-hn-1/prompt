"""Independent optional terminal predicate. Parent types/helpers unchanged;
search body differs only by optional hook parameter and goal admission condition.
Necessary screening, not a physical or stopping certificate.
"""
import supported_four_point_planning as _parent
for _key, _value in vars(_parent).items():
    if not _key.startswith("__"):
        globals()[_key] = _value

def goal_approach_allowed(x, y, yaw, direction, goal):
    if not all(math.isfinite(float(v)) for v in (x,y,yaw,*goal)):
        raise ValueError("finite nominal node and known goal required")
    dx,dy=float(goal[0])-x,float(goal[1])-y
    squared=dx*dx+dy*dy
    if squared<=1e-24:
        return True
    if direction!="forward":
        return False
    forward=math.cos(yaw)*dx+math.sin(yaw)*dy
    lateral=-math.sin(yaw)*dx+math.cos(yaw)*dy
    return forward>0 and -.827225<=2*lateral/squared<=.796603

def plan_kinodynamic_path(
    request: KinodynamicPlanRequest, drive_model: DriveModel, *,
    initial_direction: DriveDirection | None = None,
    primitive_curvature_source=None,
    primitive_requests=None,
    goal_acceptance=None,
) -> KinodynamicPlanResult:
    """Search measured forward/reverse primitives on an inflated occupancy map."""

    if initial_direction not in (None, "forward", "reverse"):
        raise ValueError("initial_direction must be None, forward or reverse")
    if not isinstance(request, KinodynamicPlanRequest):
        raise TypeError("request must be a KinodynamicPlanRequest")
    if not isinstance(drive_model, DriveModel):
        raise TypeError("drive_model must be a DriveModel")
    if not _valid_cell(request, request.start_xy):
        return KinodynamicPlanResult("invalid_start", (), 0, 0.0, 0)
    if not _valid_cell(request, request.goal_xy):
        return KinodynamicPlanResult("invalid_goal", (), 0, 0.0, 0)

    start_cell = world_to_cell(
        request.start_xy,
        origin_xy=request.origin_xy,
        resolution_m=request.resolution_m,
    )
    start_yaw_bin = yaw_to_bin(request.start_yaw_rad)
    goal_cell = world_to_cell(
        request.goal_xy,
        origin_xy=request.origin_xy,
        resolution_m=request.resolution_m,
    )
    grid_distance = _grid_distance_to_goal(
        request.occupancy_grid,
        goal_cell=goal_cell,
        resolution_m=request.resolution_m,
        occupied=request.occupied,
    )

    def heuristic(x: float, y: float) -> float:
        row = int(math.floor((x - request.origin_xy[0]) / request.resolution_m + 0.5))
        column = int(math.floor((y - request.origin_xy[1]) / request.resolution_m + 0.5))
        if 0 <= row < grid_distance.shape[0] and 0 <= column < grid_distance.shape[1]:
            obstacle_aware = float(grid_distance[row, column])
            if math.isfinite(obstacle_aware):
                return max(0.0, obstacle_aware - request.goal_tolerance_m)
        return max(
            0.0,
            math.hypot(x - request.goal_xy[0], y - request.goal_xy[1])
            - request.goal_tolerance_m,
        )

    primitive_templates: list[
        tuple[DriveDirection, float, float, np.ndarray]
    ] = []
    if primitive_requests is None:
        commands = tuple((sign*magnitude, auxiliary) for sign in (1.,-1.) for magnitude in MAIN_MAGNITUDES_RAD_S for auxiliary in AUXILIARY_ANGLES_RAD)
    else:
        if primitive_curvature_source is None:
            raise ValueError('explicit supported commands require an explicit curvature source')
        commands = tuple(primitive_requests)
        if not commands or any(len(c)!=2 or not all(math.isfinite(v) for v in c) or c[0]==0 for c in commands):
            raise ValueError('finite nonzero supported primitive requests required')
    for main, auxiliary in commands:
        sign = 1. if main > 0 else -1.
        direction = "forward" if sign > 0 else "reverse"
        curvature = drive_model.curvature(main, auxiliary) if primitive_curvature_source is None else primitive_curvature_source(main, auxiliary)
        if curvature is None:
            continue
        if not math.isfinite(curvature):
            raise ValueError('finite supported primitive curvature')
        primitive_templates.append((direction, main, auxiliary, sample_motion_primitive(
            start_pose=(0.0, 0.0, 0.0), signed_arc_length_m=sign*request.primitive_length_m,
            curvature_1_m=curvature, sample_spacing_m=request.collision_sample_spacing_m)))

    tie = itertools.count()
    frontier: list[tuple[float, int, StateKey]] = []
    costs: dict[StateKey, float] = {}
    parents: dict[StateKey, StateKey | None] = {}
    poses: dict[StateKey, tuple[float, float, float]] = {}
    incoming_actions: dict[StateKey, tuple[float, float]] = {}
    for direction in ("forward", "reverse"):
        key: StateKey = (*start_cell, start_yaw_bin, direction)
        costs[key] = (request.direction_change_cost_m
                      if initial_direction is not None and direction != initial_direction
                      else 0.0)
        parents[key] = None
        poses[key] = (*request.start_xy, request.start_yaw_rad)
        heapq.heappush(
            frontier,
            (costs[key] + heuristic(*request.start_xy), next(tie), key),
        )

    closed: set[StateKey] = set()
    origin_array = np.asarray(request.origin_xy, dtype=np.float64)
    expanded = 0
    while frontier:
        _, _, current = heapq.heappop(frontier)
        if current in closed:
            continue
        if expanded >= request.node_budget:
            return KinodynamicPlanResult(
                "node_budget_exhausted", (), expanded, 0.0, 0
            )
        closed.add(current)
        expanded += 1
        x, y, yaw = poses[current]
        if math.hypot(x - request.goal_xy[0], y - request.goal_xy[1]) <= (
            request.goal_tolerance_m + 1e-12
        ) and (goal_acceptance is None or goal_acceptance(x, y, yaw, current[3])):
            return _reconstruct(
                current,
                parents=parents,
                poses=poses,
                incoming_actions=incoming_actions,
                primitive_length_m=request.primitive_length_m,
                expanded_nodes=expanded,
            )

        for direction, main, auxiliary, local_samples in primitive_templates:
            samples = _transform_local_primitive(
                local_samples, x_m=x, y_m=y, yaw_rad=yaw
            )
            if not _primitive_is_collision_free_unchecked(
                samples,
                grid=request.occupancy_grid,
                origin_xy=origin_array,
                resolution_m=request.resolution_m,
                occupied=request.occupied,
            ):
                continue
            next_x, next_y, next_yaw = (float(value) for value in samples[-1])
            row, column = world_to_cell(
                (next_x, next_y),
                origin_xy=request.origin_xy,
                resolution_m=request.resolution_m,
            )
            neighbor: StateKey = (
                row,
                column,
                yaw_to_bin(next_yaw),
                direction,
            )
            edge_cost = request.primitive_length_m
            if direction == "reverse":
                edge_cost *= request.reverse_cost_multiplier
            if direction != current[3]:
                edge_cost += request.direction_change_cost_m
            candidate = costs[current] + edge_cost
            if candidate + 1e-12 >= costs.get(neighbor, math.inf):
                continue
            costs[neighbor] = candidate
            parents[neighbor] = current
            poses[neighbor] = (next_x, next_y, next_yaw)
            incoming_actions[neighbor] = (main, auxiliary)
            heapq.heappush(
                frontier,
                (candidate + heuristic(next_x, next_y), next(tie), neighbor),
            )

    return KinodynamicPlanResult("unreachable", (), expanded, 0.0, 0)
