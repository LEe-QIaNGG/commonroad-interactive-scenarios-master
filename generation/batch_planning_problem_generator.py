import argparse
import os
import re
from typing import Optional, Tuple

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.common.solution import (
    CommonRoadSolutionWriter,
    CostFunction,
    PlanningProblemSolution,
    Solution,
    VehicleModel,
    VehicleType,
)
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.state import KSState
from commonroad.scenario.trajectory import Trajectory
from commonroad_velocity_planner.global_trajectory import GlobalTrajectory
from commonroad_velocity_planner.velocity_planner_interface import ImplementedPlanners

from commonroad_global_planner.global_planner import GlobalPlanner
from planning_problem import generate_random_planning_problems
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="读取 CommonRoad cr.xml，为单个车辆迭代生成可规划的规划问题并保存。"
    )
    parser.add_argument(
        "--input",
        required=True,
        help="输入的 .cr.xml 文件路径",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="随机种子（会同时影响输出文件命名）",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="输出目录（默认写入输入文件目录下的 scenario-{seed} 子目录）",
    )
    parser.add_argument(
        "--max-attempts",
        type=int,
        default=50,
        help="生成并检测规划问题的最大尝试次数，避免死循环",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="若目标文件存在，是否直接覆盖",
    )
    return parser.parse_args()


def plan_trajectory(
    scenario, planning_problem: PlanningProblem
) -> Optional[GlobalTrajectory]:
    global_planner: GlobalPlanner = GlobalPlanner(
        scenario=scenario,
        planning_problem=planning_problem,
    )
    candidate_trajectory: GlobalTrajectory = global_planner.plan_global_trajectory(
        use_regulatory_stop_elements=True,
        regulatory_elements_time_step=0,
        retrieve_shortest=True,
        consider_least_lance_changes=True,
        velocity_planner=ImplementedPlanners.BangBangSTPlanner,
    )
    if (
        candidate_trajectory is not None
        and hasattr(candidate_trajectory, "reference_path")
        and len(candidate_trajectory.reference_path) > 2
    ):
        return candidate_trajectory
    return None


def _replace_suffix_with_seed(name: str, seed: Optional[int]) -> str:
    if seed is None:
        return f"{name}_generated"
    match = re.search(r"-(\d+)$", name)
    if match:
        return f"{name[: match.start()]}-{seed}"
    return f"{name}-{seed}"


def build_output_path(
    input_path: str, seed: Optional[int], output_dir: Optional[str]
) -> str:
    abs_input = os.path.abspath(input_path)
    input_dir = os.path.dirname(abs_input)
    parent_base = os.path.basename(input_dir)
    parent_parent = os.path.dirname(input_dir)

    if output_dir:
        target_dir = os.path.abspath(output_dir)
    else:
        folder_name = _replace_suffix_with_seed(parent_base, seed)
        target_dir = os.path.join(parent_parent, folder_name)

    os.makedirs(target_dir, exist_ok=True)

    input_name = os.path.basename(abs_input)
    if input_name.endswith(".cr.xml"):
        stem = input_name[: -len(".cr.xml")]
        new_stem = _replace_suffix_with_seed(stem, seed)
        output_name = f"{new_stem}.cr.xml"
    else:
        suffix = f"_{seed}" if seed is not None else "_generated"
        output_name = f"{input_name}{suffix}.cr.xml"
    return os.path.join(target_dir, output_name)


def write_planning_problems(
    scenario, planning_problem_set, output_path: str, overwrite: bool
) -> None:
    author = "BatchGenerator"
    affiliation = "CommonRoad Interactive Scenarios"
    source = "Generated via batch_planning_problem_generator.py"
    tags = scenario.tags

    writer = CommonRoadFileWriter(
        scenario,
        planning_problem_set,
        author,
        affiliation,
        source,
        tags,
    )
    overwrite_policy = (
        OverwriteExistingFile.ALWAYS if overwrite else OverwriteExistingFile.ASK_USER
    )
    writer.write_to_file(output_path, overwrite_policy)
    print(f"规划问题已写入: {output_path}")


def convert_to_commonroad_trajectory(
    planning_problem: PlanningProblem, global_trajectory: GlobalTrajectory
) -> Trajectory:
    state_list = []
    ref_path = global_trajectory.reference_path
    orientations = getattr(global_trajectory, "path_orientation", None)
    velocities = getattr(global_trajectory, "velocity_profile", None)

    initial_state = planning_problem.initial_state.convert_state_to_state(KSState())
    if getattr(initial_state, "steering_angle", None) is None:
        initial_state.steering_angle = 0.0
    state_list.append(initial_state)

    for idx, position in enumerate(ref_path):
        orientation = float(orientations[idx]) if orientations is not None else 0.0
        velocity = float(velocities[idx]) if velocities is not None else 0.0
        time_step = int(initial_state.time_step + idx + 1)
        state = KSState(
            position=np.array(position),
            steering_angle=0.0,
            orientation=orientation,
            velocity=velocity,
            time_step=time_step,
        )
        state_list.append(state)

    return Trajectory(initial_time_step=initial_state.time_step, state_list=state_list)


def write_solution_file(
    scenario,
    planning_problem: PlanningProblem,
    global_trajectory: GlobalTrajectory,
    output_path: str,
    overwrite: bool,
) -> None:
    trajectory = convert_to_commonroad_trajectory(planning_problem, global_trajectory)
    solution = Solution(
        scenario_id=scenario.scenario_id,
        planning_problem_solutions=[
            PlanningProblemSolution(
                planning_problem_id=planning_problem.planning_problem_id,
                vehicle_type=VehicleType.BMW_320i,
                vehicle_model=VehicleModel.KS,
                cost_function=CostFunction.JB1,
                trajectory=trajectory,
            )
        ],
    )
    writer = CommonRoadSolutionWriter(solution)
    solution_dir = os.path.dirname(output_path) or "."
    os.makedirs(solution_dir, exist_ok=True)
    writer.write_to_file(
        output_path=solution_dir,
        filename=os.path.basename(output_path),
        overwrite=overwrite,
    )
    print(f"规划轨迹已写入: {output_path}")


def build_solution_output_path(
    planning_output_path: str,
) -> str:
    base, _ = os.path.splitext(planning_output_path)
    if base.endswith(".cr"):
        base = base[:-3]
    return f"{base}_solution.xml"


def find_feasible_planning_problem(
    scenario,
    base_seed: Optional[int],
    max_attempts: int,
) -> Tuple[PlanningProblemSet, PlanningProblem, GlobalTrajectory]:
    for attempt in range(max_attempts):
        attempt_seed = base_seed + attempt if base_seed is not None else None
        planning_problem_set = generate_random_planning_problems(
            scenario, num_problems=1, random_seed=attempt_seed
        )
        if len(planning_problem_set.planning_problem_dict) == 0:
            print(f"第 {attempt + 1} 次尝试：未生成有效的 PlanningProblem，继续。")
            continue
        planning_problem = next(
            iter(planning_problem_set.planning_problem_dict.values())
        )
        try:
            trajectory = plan_trajectory(scenario, planning_problem)
            if trajectory is not None:
                print(
                    f"第 {attempt + 1} 次尝试成功，生成的 PlanningProblem 可规划，轨迹长度 {len(trajectory.reference_path)}"
                )
                return planning_problem_set, planning_problem, trajectory
            print(f"第 {attempt + 1} 次尝试规划失败，继续。")
        except Exception as exc:
            print(f"第 {attempt + 1} 次尝试规划异常：{exc}，继续。")

    raise RuntimeError(
        f"在 {max_attempts} 次尝试内未能得到可规划的规划问题，请调整参数或场景后重试。"
    )


def main():
    args = parse_args()
    input_path = os.path.abspath(args.input)
    if not os.path.isfile(input_path):
        raise FileNotFoundError(f"找不到输入文件: {input_path}")

    scenario, _ = CommonRoadFileReader(input_path).open()
    (
        planning_problem_set,
        planning_problem,
        trajectory,
    ) = find_feasible_planning_problem(scenario, args.seed, args.max_attempts)

    output_path = build_output_path(input_path, args.seed, args.output_dir)
    write_planning_problems(scenario, planning_problem_set, output_path, args.overwrite)

    solution_output_path = build_solution_output_path(output_path)
    write_solution_file(
        scenario,
        planning_problem,
        trajectory,
        solution_output_path,
        args.overwrite,
    )


if __name__ == "__main__":
    main()

