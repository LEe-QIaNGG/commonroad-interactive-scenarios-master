import argparse
import os
import re
from typing import List, Optional, Tuple

import numpy as np
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

from planning_problem import generate_validated_planning_problems


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
        "--num-vehicle",
        type=int,
        default=1,
        help="需要生成轨迹的规划问题数量",
    )
    parser.add_argument(
        "--num-ego",
        type=int,
        default=1,
        help="写入 cr.xml 并作为 ego vehicle 使用的规划问题数量",
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
    planning_problem_trajectories: List[Tuple[PlanningProblem, Trajectory]],
    output_path: str,
    overwrite: bool,
) -> None:
    if not planning_problem_trajectories:
        print("未生成轨迹，跳过 Solution 文件写入。")
        return

    solution_entries = []
    for planning_problem, trajectory in planning_problem_trajectories:
        solution_entries.append(
            PlanningProblemSolution(
                planning_problem_id=planning_problem.planning_problem_id,
                vehicle_type=VehicleType.BMW_320i,
                vehicle_model=VehicleModel.KS,
                cost_function=CostFunction.JB1,
                trajectory=trajectory,
            )
        )

    solution = Solution(
        scenario_id=scenario.scenario_id,
        planning_problem_solutions=solution_entries,
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


def main():
    args = parse_args()
    input_path = os.path.abspath(args.input)
    if not os.path.isfile(input_path):
        raise FileNotFoundError(f"找不到输入文件: {input_path}")

    if args.num_ego <= 0:
        raise ValueError("--num-ego 必须大于 0")
    if args.num_vehicle < args.num_ego:
        raise ValueError("--num-vehicle 不能小于 --num-ego")

    scenario, _ = CommonRoadFileReader(input_path).open()
    planning_problem_set, planning_problem_solutions = generate_validated_planning_problems(
        scenario,
        num_vehicle=args.num_vehicle,
        num_additional=0,
        random_seed=args.seed,
        max_attempts=args.max_attempts,
    )

    output_path = build_output_path(input_path, args.seed, args.output_dir)

    planning_problem_trajectories: List[Tuple[PlanningProblem, Trajectory]] = []
    for planning_problem, global_trajectory in planning_problem_solutions:
        trajectory = convert_to_commonroad_trajectory(
            planning_problem, global_trajectory
        )
        planning_problem_trajectories.append((planning_problem, trajectory))

    sorted_trajectories = sorted(
        planning_problem_trajectories, key=lambda item: item[0].planning_problem_id
    )
    ego_trajectories = sorted_trajectories[: args.num_ego]

    planning_problem_set_to_write = PlanningProblemSet()
    for planning_problem, _ in ego_trajectories:
        planning_problem_set_to_write.add_planning_problem(planning_problem)

    write_planning_problems(
        scenario,
        planning_problem_set_to_write,
        output_path,
        args.overwrite,
    )

    solution_output_path = build_solution_output_path(output_path)
    write_solution_file(
        scenario,
        sorted_trajectories,
        solution_output_path,
        args.overwrite,
    )

if __name__ == "__main__":
    main()

