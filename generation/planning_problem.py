
from typing import List, Optional, Tuple

import numpy as np
from commonroad.common.util import AngleInterval, Interval
from commonroad.geometry.shape import Circle
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.state import CustomState, InitialState
from commonroad_velocity_planner.global_trajectory import GlobalTrajectory
from commonroad_velocity_planner.velocity_planner_interface import (
    ImplementedPlanners,
)

from commonroad_global_planner.global_planner import GlobalPlanner
from generate_vehicle import generate_random_position_and_orientation


def _create_planning_problem(
    scenario, planning_problem_id: int
) -> Optional[PlanningProblem]:
    start_pos, start_orientation = generate_random_position_and_orientation(scenario)
    if start_pos is None or start_orientation is None:
        return None

    goal_pos, goal_orientation = generate_random_position_and_orientation(scenario)
    if goal_pos is None or goal_orientation is None:
        return None

    initial_state = InitialState(
        position=start_pos,
        orientation=start_orientation,
        velocity=5.0,
        acceleration=0.0,
        time_step=0,
        yaw_rate=0.0,
        slip_angle=0.0,
    )

    goal_state = CustomState(
        position=Circle(5.0, goal_pos),
        orientation=AngleInterval(0.0, 0.0),
        velocity=Interval(0.0, 0.0),
        time_step=Interval(100, 100),
    )
    goal_region = GoalRegion([goal_state])

    return PlanningProblem(
        planning_problem_id=planning_problem_id,
        initial_state=initial_state,
        goal_region=goal_region,
    )


def generate_random_planning_problems(
    scenario, num_problems: int = 5, random_seed: Optional[int] = None, start_id: int = 0
) -> PlanningProblemSet:
    planning_problem_set = PlanningProblemSet()
    if random_seed is not None:
        np.random.seed(random_seed)

    generated = 0
    attempts = 0
    max_attempts = num_problems * 10 if num_problems > 0 else 10

    while generated < num_problems and attempts < max_attempts:
        attempts += 1
        planning_problem = _create_planning_problem(
            scenario, start_id + len(planning_problem_set.planning_problem_dict)
        )
        if planning_problem is None:
            continue
        planning_problem_set.add_planning_problem(planning_problem)
        generated += 1

    if generated < num_problems:
        print(f"仅生成 {generated}/{num_problems} 个规划问题。")

    return planning_problem_set


def _plan_with_global_planner(
    scenario, planning_problem: PlanningProblem
) -> Optional[GlobalTrajectory]:
    try:
        global_planner = GlobalPlanner(
            scenario=scenario,
            planning_problem=planning_problem,
        )
        candidate = global_planner.plan_global_trajectory(
            use_regulatory_stop_elements=True,
            regulatory_elements_time_step=0,
            retrieve_shortest=True,
            consider_least_lance_changes=True,
            velocity_planner=ImplementedPlanners.BangBangSTPlanner,
        )
        if (
            candidate is not None
            and hasattr(candidate, "reference_path")
            and len(candidate.reference_path) > 2
        ):
            return candidate
    except Exception as exc:
        print(
            f"PlanningProblem {planning_problem.planning_problem_id} 规划异常：{exc}"
        )
    return None


def generate_validated_planning_problems(
    scenario,
    num_vehicle: int,
    num_additional: int = 0,
    random_seed: Optional[int] = None,
    max_attempts: int = 200,
    start_id: int = 0,
) -> Tuple[PlanningProblemSet, List[Tuple[PlanningProblem, GlobalTrajectory]]]:
    total_required = num_vehicle + num_additional
    if total_required <= 0:
        raise ValueError("num_vehicle + num_additional 必须大于 0")

    planning_problem_set = PlanningProblemSet()
    feasible_solutions: List[Tuple[PlanningProblem, GlobalTrajectory]] = []
    attempts = 0

    while (
        len(planning_problem_set.planning_problem_dict) < total_required
        or len(feasible_solutions) < num_vehicle
    ):
        if attempts >= max_attempts:
            raise RuntimeError(
                "在最大尝试次数内未能生成足够的可规划问题，请调整参数或场景。"
            )
        attempt_seed = (
            (random_seed + attempts) if random_seed is not None else None
        )
        if attempt_seed is not None:
            np.random.seed(attempt_seed)
        attempts += 1

        next_id = start_id + len(planning_problem_set.planning_problem_dict)
        planning_problem = _create_planning_problem(scenario, next_id)
        if planning_problem is None:
            continue

        need_feasible = len(feasible_solutions) < num_vehicle
        trajectory = (
            _plan_with_global_planner(scenario, planning_problem)
            if need_feasible
            else None
        )
        if need_feasible and trajectory is None:
            continue

        planning_problem_set.add_planning_problem(planning_problem)
        if need_feasible and trajectory is not None:
            feasible_solutions.append((planning_problem, trajectory))

    return planning_problem_set, feasible_solutions

