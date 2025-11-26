import copy
from pathlib import Path
from typing import Literal

from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.visualization.draw_params import (
    BaseParam,
    DynamicObstacleParams,
    MPDrawParams,
)
from commonroad.visualization.drawable import IDrawable
from commonroad.visualization.mp_renderer import MPRenderer

from commonroad_sumo.errors import SumoInterfaceError
from commonroad_sumo.utils import (
    get_scenario_final_time_step,
    get_scenario_start_time_step,
)

# Control the dimensions of the resulting video.
_DEFAULT_VIDEO_FIG_SIZE = [20, 10]
_DEFAULT_VIDEO_DPI = 120


def create_video_with_ego_vehicles(
    output_folder: Path | str,
    scenario: Scenario,
    ego_vehicle_obstacles: list[DynamicObstacle],
    planning_problem_set: PlanningProblemSet | None = None,
    follow_ego: bool = False,
    video_file_type: Literal["mp4", "gif"] = "gif",
) -> Path:
    """
    Create video for a simulated scenario and the list of ego vehicles.

    :param output_folder: Path to a folder where the resulting video will be stored.
    :param scenario: The simulated CommonRoad scenario.
    :param ego_vehicle_obstacles: The ego vehicles as `DynamicObstacles`.
    :param planning_problem_set: Optionally, the planning problem set for the ego vehicle(s).
    :param follow_ego: focus video on the ego vehicle(s)
    :param follow_ego: If enabled the video will focus the ego vehicle during its movement. If more than one ego vehicle is present, the video creation will fail.
    :param video_file_type: Select resulting video file format. Valid formats are `mp4` and `gif`.

    :returns: The path of the created video.

    :raises SumoInterfaceError: If `follow_ego` is enabled, but multiple ego vehicles exist in the planning problem set.
    """
    # The renderer does not automatically determine the time horizon, so it must be done manually.
    time_begin = get_scenario_start_time_step(scenario)
    time_end = get_scenario_final_time_step(scenario)

    # Create the params for dynamic obstacles. They are created here, so that they can be reused
    # for the ego vehicles so that the visualization stays consistent.
    base_dynamic_obstacle_draw_params = DynamicObstacleParams()
    base_dynamic_obstacle_draw_params.time_begin = time_begin
    base_dynamic_obstacle_draw_params.time_end = time_end
    base_dynamic_obstacle_draw_params.show_label = True
    base_dynamic_obstacle_draw_params.draw_icon = True
    base_dynamic_obstacle_draw_params.draw_shape = True

    draw_params = MPDrawParams(
        time_begin=time_begin,
        time_end=time_end,
        dynamic_obstacle=base_dynamic_obstacle_draw_params,
        # If `follow_ego` is active the axis would stutter during the movement of the ego vehicle.
        axis_visible=not follow_ego,
    )

    rnd = MPRenderer()

    # If follow_ego is enabled the renderer should focus on the single ego vehicle.
    if follow_ego:
        if len(ego_vehicle_obstacles) != 1:
            raise SumoInterfaceError(
                f"Cannot create video for scenario {scenario.scenario_id}: If `follow_ego` is enabled only one ego vehicle must be given, but {len(ego_vehicle_obstacles)} ego vehicles are given (ego vehicles: {', '.join([str(obstacle.obstacle_id) for obstacle in ego_vehicle_obstacles])})"
            )

        rnd.focus_obstacle_id = ego_vehicle_obstacles[0].obstacle_id

    # For `MPRenderer.create_video`, we must explicitly pass a collection of objects that should be rendered.
    # Since we require specific draw params for each ego vehicle, we must additionally create a draw params list.
    # The indices between `rnd_obj_list` and `draw_params_list` must match, so that the
    # draw params can be correctly mapped to each object by `MPRenderer`.
    rnd_obj_list: list[IDrawable] = [scenario]
    draw_params_list: list[BaseParam] = [draw_params]

    # Ego vehicles are rendered similar to all other vehicles, but are additionally highlighted.
    ego_vehicle_obstacle_draw_params = copy.deepcopy(base_dynamic_obstacle_draw_params)
    ego_vehicle_obstacle_draw_params.occupancy.shape.facecolor = "green"
    ego_vehicle_obstacle_draw_params.trajectory.draw_trajectory = True
    for ego_vehicle_obstacle in ego_vehicle_obstacles:
        rnd_obj_list.append(ego_vehicle_obstacle)
        draw_params_list.append(ego_vehicle_obstacle_draw_params)

    # If the planning problem set is given it should also be rendered.
    if planning_problem_set is not None:
        rnd_obj_list.append(planning_problem_set)
        draw_params_list.append(draw_params)

    # Determine the video file location.
    if not isinstance(output_folder, Path):
        output_folder = Path(output_folder)
    file_path = (output_folder / str(scenario.scenario_id)).with_suffix(
        f".{video_file_type}"
    )

    # Finally, create the video with the ego vehicles, planning problem set and scenario.
    rnd.create_video(
        rnd_obj_list,
        str(file_path),
        draw_params=draw_params_list,
        fig_size=_DEFAULT_VIDEO_FIG_SIZE,
        dpi=_DEFAULT_VIDEO_DPI,
        # Explicitly set the dt in case the `MPRenderer` cannot auto-detect it.
        # dt must be in [ms], but scenario dt is in [s]. Therefore, it must be scaled with 1000.
        dt=scenario.dt * 1000.0,
    )
    return file_path
