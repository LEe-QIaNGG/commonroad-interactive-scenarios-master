import copy
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Literal, Optional, TypedDict, Union

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.solution import (
    CostFunction,
    PlanningProblemSolution,
    Solution,
    VehicleModel,
    VehicleType,
)
from commonroad.planning.planner_interface import TrajectoryPlannerInterface
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.obstacle import (
    DynamicObstacle,
    ObstacleType,
    Rectangle,
    TrajectoryPrediction,
)
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InitialState
from commonroad.scenario.trajectory import Trajectory
from typing_extensions import Self, Unpack, override

from commonroad_sumo.backend.sumo_simulation_backend import SumoSimulationBackend
from commonroad_sumo.cr2sumo import (
    CR2SumoMapConverter,
    ResimulationTrafficGeneratorConfig,
    UnsafeResimulationTrafficGenerator,
)
from commonroad_sumo.errors import (
    SumoInteractivePlanningFailedError,
    SumoInterfaceError,
)
from commonroad_sumo.helpers import create_new_scenario_with_metadata_from_old_scenario
from commonroad_sumo.interface.id_mapper import IdMapper
from commonroad_sumo.simulation.abstract_sumo_simulation import (
    AbstractSumoSimulation,
    SumoSimulationConfig,
    SumoSimulationResult,
)
from commonroad_sumo.sumolib.sumo_project import SumoProject
from commonroad_sumo.utils import (
    convert_state_to_state_type,
    get_full_trajectory_of_obstacle,
)
from commonroad_sumo.visualization import create_video_with_ego_vehicles

_LOGGER = logging.getLogger(__name__)


@dataclass
class InteractiveSumoSimulationResult(SumoSimulationResult):
    """
    Result of an interactive simulation.
    """

    planning_problem_set: PlanningProblemSet
    ego_vehicles: dict[int, DynamicObstacle]

    @override
    def write_to_file(self, file_path: Union[str, Path]) -> None:
        if isinstance(file_path, Path):
            file_path = str(file_path.absolute())
        CommonRoadFileWriter(self.scenario, PlanningProblemSet()).write_to_file(
            file_path
        )

    def get_scenario_with_ego_vehicles_as_dynamic_obstacles(self) -> Scenario:
        """
        Returns the simulated scenario and adds all ego vehicles as dynamic obstacles to the scenario.
        """
        new_scenario = copy.deepcopy(self.scenario)
        new_scenario.add_objects(list(self.ego_vehicles.values()))
        return new_scenario

    def get_planning_problem_solution(
        self,
        planning_problem: PlanningProblem,
        vehicle_model: VehicleModel = VehicleModel.PM,
        vehicle_type: VehicleType = VehicleType.FORD_ESCORT,
        cost_function: CostFunction = CostFunction.JB1,
    ) -> PlanningProblemSolution:
        """
        Retrive the solution for a given planning problem.
        """
        ego_vehicle = self.ego_vehicles[planning_problem.planning_problem_id]
        trajectory = get_full_trajectory_of_obstacle(ego_vehicle)
        return PlanningProblemSolution(
            planning_problem_id=planning_problem.planning_problem_id,
            vehicle_model=vehicle_model,
            vehicle_type=vehicle_type,
            cost_function=cost_function,
            trajectory=trajectory,
        )

    def get_planning_problem_solutions(
        self,
        vehicle_model: VehicleModel = VehicleModel.PM,
        vehicle_type: VehicleType = VehicleType.FORD_ESCORT,
        cost_function: CostFunction = CostFunction.JB1,
    ) -> List[PlanningProblemSolution]:
        """
        Retrive solutions to all planning problems.
        """
        return [
            self.get_planning_problem_solution(
                planning_problem, vehicle_model, vehicle_type, cost_function
            )
            for planning_problem in self.planning_problem_set.planning_problem_dict.values()
        ]

    def get_solution(
        self,
        vehicle_model: VehicleModel = VehicleModel.PM,
        vehicle_type: VehicleType = VehicleType.FORD_ESCORT,
        cost_function: CostFunction = CostFunction.JB1,
    ) -> Solution:
        """
        Retrive the solution object for the scenario.

        :param vehicle_model: `VehicleModel` used for the construction of each `PlanningProblemSolution`.
        :param vehicle_type: `VehicleType` used for the construction of each `PlanningProblemSolution`.
        :param cost_function: `CostFunction` used for the construction of each `PlanningProblemSolution`.

        :returns: The solution for the scenario.
        """
        planning_problem_solutions = self.get_planning_problem_solutions(
            vehicle_model, vehicle_type, cost_function
        )
        return Solution(self.scenario.scenario_id, planning_problem_solutions)

    def create_video(
        self,
        output_folder: Path | str,
        follow_ego: bool = True,
        video_file_type: Literal["mp4", "gif"] = "gif",
    ) -> Path:
        """
        Create a video for the result of an interactive simulation.

        :param output_folder: Folder where the video should be created. File is named according to the scenario.
        :param follow_ego: If enabled the video will focus the ego vehicle during its movement. If more than one ego vehicle is present in the planning problem set, the video creation will fail.
        :param video_file_type: Select video file format.

        :returns: The path of the created video.

        :raises SumoInterfaceError: If `follow_ego` is enabled, but multiple ego vehicles exist in the planning problem set.
        """
        ego_vehicle_obstacles = list(self.ego_vehicles.values())
        return create_video_with_ego_vehicles(
            output_folder,
            self.scenario,
            ego_vehicle_obstacles,
            self.planning_problem_set,
            follow_ego=follow_ego,
            video_file_type=video_file_type,
        )


PlanningProblemId = int


@dataclass
class InteractiveSumoSimulationWithMotionPlannerConfig(SumoSimulationConfig):
    ego_veh_width: float = 1.6
    ego_veh_length: float = 4.3


class InteractiveSumoSimulationWithMotionPlannerExtraArguments(TypedDict, total=False):
    simulation_config: InteractiveSumoSimulationWithMotionPlannerConfig | None
    simulation_backend: SumoSimulationBackend | None


class InteractiveSumoSimulationWithMotionPlanner(AbstractSumoSimulation):
    """
    Evaluate motion planners against a SUMO simulation.

    :param scenario: Initialize with a non-interactive CommonRoad scenario. The scenario is copied and reduced to an interactive scenario.
    :param planning_problem_set: The planning problem set which should be solved with the motion planner.
    :param sumo_project: Existing SUMO Project, usually created by the `CR2SumoMapConverter`.
    :param simulation_config: Provide an optional config. If `None` is given, the default is used.
    :param simulation_backend: Provide an optional backend, e.g., `TraciSumoSimulationBackend`. If `None` is given, the default (`LibsumoSumoSimulationBackend`) is used.
    """

    def __init__(
        self,
        scenario: Scenario,
        planning_problem_set: PlanningProblemSet,
        sumo_project: SumoProject,
        simulation_config: Optional[
            InteractiveSumoSimulationWithMotionPlannerConfig
        ] = None,
        simulation_backend: Optional[SumoSimulationBackend] = None,
    ) -> None:
        if simulation_config is None:
            simulation_config = InteractiveSumoSimulationWithMotionPlannerConfig()

        interactive_scenario = _reduce_scenario_to_interactive_scenario(scenario)
        # A pre-conditioned IdMapper is needed for the interactive simulation.
        # It must already contain an ID mapping for all dynamic obstacles,
        # since they will only be synced *from* the simulation and never *to* the simulation.
        # If the IdMapper is not pre-conditioned with the necessary mappings, some obstacles
        # cannot be synced from the simulation.
        id_mapper = _create_id_mapper_for_interactive_scenario(interactive_scenario)
        super().__init__(
            interactive_scenario,
            sumo_project,
            id_mapper=id_mapper,
            simulation_config=simulation_config,
            simulation_backend=simulation_backend,
        )

        self._simulation_config = simulation_config
        self._planning_problem_set = copy.deepcopy(planning_problem_set)
        self._ego_vehicles: dict[PlanningProblemId, DynamicObstacle] = (
            self._create_ego_vehicles_for_planning_problem_set(
                self._scenario, self._planning_problem_set
            )
        )

        self._max_simulation_steps: int = 0
        self._reevaluation_interval: int | None = None
        self._motion_planner: Optional[TrajectoryPlannerInterface] = None
        self._dirty: bool = False

    @classmethod
    def from_scenario(
        cls,
        scenario: Scenario,
        planning_problem_set: PlanningProblemSet,
        **kwargs: Unpack[InteractiveSumoSimulationWithMotionPlannerExtraArguments],
    ) -> Self:
        converter = CR2SumoMapConverter(scenario)
        sumo_project = converter.create_sumo_files()
        if sumo_project is None:
            raise SumoInterfaceError(
                f"Failed to convert {scenario.scenario_id} to SUMO!"
            )

        # The resimulation traffic generator tries to resimulate scenarios as close as possible by default.
        # However, for the interactive simulation we have a bit more freedom, since we do not aim
        # for a 1:1 resimulation. With route extension enabled, we get much better simulation
        # results, although with the tradeoff of some reduced accuracy regarding the original scenario.
        traffic_generator_config = ResimulationTrafficGeneratorConfig(
            extend_routes_downstream_junction=True,
            extend_routes_upstream_junction=True,
            extend_routes=True,
        )
        # The interactive simulation is an unsafe re-simulation, because the initial states
        # should be matched as close as possible. With a safe re-simulation the initial states
        # might be delayed.
        traffic_generator = UnsafeResimulationTrafficGenerator(
            config=traffic_generator_config
        )
        traffic_generator.generate_traffic(scenario, sumo_project)

        return cls(scenario, planning_problem_set, sumo_project, **kwargs)

    @classmethod
    def from_file(
        cls,
        file_path: Union[str, Path],
        **kwargs: Unpack[InteractiveSumoSimulationWithMotionPlannerExtraArguments],
    ) -> Self:
        scenario, planning_problem_set = CommonRoadFileReader(file_path).open()
        return cls.from_scenario(scenario, planning_problem_set, **kwargs)

    def _create_ego_vehicle_from_planning_problem(
        self, interactive_scenario: Scenario, planning_problem: PlanningProblem
    ) -> DynamicObstacle:
        """
        Create a new DynamicObstacle based on the :param:`planning_problem`. The :param:`interactive_scenario` is required for the CommonRoad ID generation.

        :param interactive_scenario: The base interactive scenario, used only for ID generation.
        :param planning_problem: The planning problem from which the ego vehicle will be derived
        """
        # An ego vehicle is a DynamicObstacle with the planning problem as its initial state
        ego_vehicle = DynamicObstacle(
            # Use the ID generation capabilities from CommonRoad, to make sure there are no duplicates
            obstacle_id=interactive_scenario.generate_object_id(),
            obstacle_type=ObstacleType.CAR,
            obstacle_shape=Rectangle(
                length=self._simulation_config.ego_veh_length,
                width=self._simulation_config.ego_veh_width,
            ),
            initial_state=planning_problem.initial_state,
        )
        return ego_vehicle

    def _create_ego_vehicles_for_planning_problem_set(
        self,
        interactive_scenario: Scenario,
        planning_problem_set: PlanningProblemSet,
    ) -> Dict[int, DynamicObstacle]:
        """
        Derive new ego vehicles from the planning problems. The :param:`interactive_scenario` is required for the CommonRoad ID generation.

        :param interactive_scenario: The base interactive scenario
        :returns: A mapping of planning problem id to ego vehicle
        """
        ego_vehicles = {}
        for (
            planning_problem_id,
            planning_problem,
        ) in planning_problem_set.planning_problem_dict.items():
            ego_vehicle = self._create_ego_vehicle_from_planning_problem(
                interactive_scenario, planning_problem
            )
            ego_vehicles[planning_problem_id] = ego_vehicle
        return ego_vehicles

    def _copy_planning_problems(
        self, planning_problem_set: PlanningProblemSet
    ) -> List[PlanningProblem]:
        """
        Copy all planning problems, so that they can be modified during the simulation.
        """
        return [
            copy.deepcopy(planning_problem)
            for planning_problem in planning_problem_set.planning_problem_dict.values()
        ]

    @override
    def _reached_simulation_end_condition(self, time_step: int) -> bool:
        if time_step == self._max_simulation_steps:
            _LOGGER.debug(
                "Interactive simulation for scenario %s has been stopped, because the maximum number of simulation steps %i was reached",
                self._scenario.scenario_id,
                self._max_simulation_steps,
            )
            return True

        if _did_all_ego_vehicles_reach_their_goal_region(
            self._ego_vehicles, self._planning_problem_set, max(time_step - 1, 0)
        ):
            _LOGGER.debug(
                "Interactive simulation for scenario %s has been stopped, because all ego vehicles reached their goal region",
                self._scenario.scenario_id,
            )
            return True

        return False

    def _execute_motion_planner(self, planning_problem: PlanningProblem) -> Trajectory:
        """Execute the motion planner on the given planning problem.

        :param planning_problem: The planning problem on which the motion planner should be executed.

        :returns: Trajectory which is already adjusted to the reevaluation interval.
        """
        if self._motion_planner is None:
            raise SumoInterfaceError(
                "Cannot perform an interactive simulation without a motion planner"
            )

        try:
            # The ego vehicle does not have trajectory set at the current time step -> execute the motion planner
            trajectory = self._motion_planner.plan(self._scenario, planning_problem)
        except Exception as exp:
            raise SumoInteractivePlanningFailedError(
                self._scenario, planning_problem
            ) from exp

        if self._reevaluation_interval is not None:
            # Cut the trajectory down to the reevalution_interval to trigger
            # a reevaluation of the motion planner in reevaluation_interval time steps
            cut_trajectory = _cut_trajectory(trajectory, self._reevaluation_interval)
        else:
            cut_trajectory = trajectory
        return cut_trajectory

    def _calibrate_vehicles_in_simulation(self, time_step: int) -> None:
        """
        Force all dynamic obstacles to move to their recorded position at the time step.
        """
        _LOGGER.debug(
            f"Calibrating dynamic obstacles in simulation at time step {time_step}"
        )
        for obstacle in self._scenario.dynamic_obstacles:
            # Only calibrate obstacles if they should exist at this time step.
            if obstacle.initial_state.time_step > time_step:
                continue

            # The calibration is allowed to fail (for now),
            # because the routes of some vehicles might still be invalid.
            try:
                self.sync_to_sumo_simulation(obstacle)
            except SumoInterfaceError as e:
                _LOGGER.debug(
                    f"failed to calibrate dynamic obstacle {obstacle.obstacle_id}: {e}"
                )

    @override
    def _pre_simulation_step_hook(self, time_step: int) -> None:
        # In the initial time step, all dynamic obstacles should be recalibrated.
        # This is done to ensure their routes follow the recorded scenario as close as possible.
        # If this is not done, the vehicles would only depart from edges but, e.g., not inside junctions.
        if time_step == 0:
            self._calibrate_vehicles_in_simulation(time_step)

        if self._motion_planner is None:
            raise SumoInterfaceError(
                "Cannot perform an interactive simulation without a motion planner"
            )

        # Perform the motion planning for every ego vehicle.
        for (
            planning_problem
        ) in self._planning_problem_set.planning_problem_dict.values():
            ego_vehicle = self._ego_vehicles[planning_problem.planning_problem_id]

            has_ego_state_at_time_step = (
                ego_vehicle.state_at_time(time_step) is not None
            )
            has_planning_problem_started = (
                planning_problem.initial_state.time_step <= time_step
            )
            # The ego vehicle should not be synced if it is not defined at the current time step.
            # An ego vehicle is considered defined at the current time step, if it either has a state
            # at the current time step (i.e. its initial state or states from previous plans),
            # or if it's associated planning problem has been started.
            if not has_ego_state_at_time_step and not has_planning_problem_started:
                # The planning problem has not yet started.
                continue

            if not has_ego_state_at_time_step:
                # The planning problem has started, but no state is defined at the ego vehicle, so we need to call the motion planner.
                _LOGGER.debug(
                    f"Executing motion planner for ego vehicle {ego_vehicle.obstacle_id} at time step {time_step}"
                )
                cut_trajectory = self._execute_motion_planner(planning_problem)

                # Adjust the planning problem, so that subsequent calls to the motion planner will plan from the end of the planned trajectory.
                new_initial_state = convert_state_to_state_type(
                    cut_trajectory.state_list[-1], InitialState
                )
                planning_problem.initial_state = new_initial_state

                _update_trajectory_prediction(ego_vehicle, cut_trajectory)

            _LOGGER.debug(
                "Syncing ego vehicle %s at time step %s",
                planning_problem.planning_problem_id,
                time_step,
            )
            # The sync might fail if the vehicle is no longer part of the simulation.
            # As we have full control over the vehicle, the vehicle should not be removed by SUMO unless explicitly instructed.
            # However, if the vehicle is not moved every time step, SUMO regains control over the vehicle
            # and might remove it at its own discretion. In such a case, we probably messed up in the previous time steps
            # and did not move it correctly, e.g., because the trajectory was wrongly cut.
            ego_sync_successful = self.sync_to_sumo_simulation(ego_vehicle)
            if not ego_sync_successful:
                raise SumoInterfaceError(
                    f"Failed to sync ego vehicle for planning problem {planning_problem.planning_problem_id} at time step {time_step} to SUMO simulation. This usually indicates a bug in the CommonRoad-SUMO interface."
                )

        # Sync the trajectories of all other traffic participants
        for obstacle in self._scenario.dynamic_obstacles:
            # Only sync obstacles if they should exist at this time step.
            if obstacle.initial_state.time_step > time_step:
                continue

            self.sync_from_sumo_simulation(obstacle)

    def run(
        self,
        motion_planner: TrajectoryPlannerInterface,
        reevaluation_interval: int | None = 1,
        max_simulation_steps: int = 500,
    ) -> InteractiveSumoSimulationResult:
        """
        Run an interactive simulation with the `motion_planner`.

        :param motion_planner: The `motion_planner` will be called continuously with an adjusted
         planning problem, either if the last trajectory was fully driven or if
         `reevaluation_interval` time steps have passed since the
         last execution of the motion planner.
        :param reevaluation_interval: An optional interval in time steps, which determines how often the motion planner should be called again, even though it returned a trajectory that is longer then the interval.
        :param max_simulation_steps: Stop the interactive simulation if not all ego vehicles did reach the goal after `max_simulation_steps`. The interactive simulation might stop earlier, if all ego vehicles reached their goal region.
        """
        if reevaluation_interval is not None and reevaluation_interval < 1:
            raise ValueError("reevaluation_interval must be greater then 0")

        if self._dirty:
            raise RuntimeError(
                "Already ran interactive simulation! Create new interactive simulation and execute again!"
            )
        else:
            self._dirty = True

        self._motion_planner = motion_planner
        self._reevaluation_interval = reevaluation_interval
        self._max_simulation_steps = max_simulation_steps

        _LOGGER.debug(
            f"Starting interactive simulation for scenario {self._scenario.scenario_id} with {len(self._ego_vehicles)} ego vehicle(s)"
        )

        self.simulation_loop()

        return InteractiveSumoSimulationResult(
            scenario=self._scenario,
            planning_problem_set=self._planning_problem_set,
            ego_vehicles=self._ego_vehicles,
        )


def _did_all_ego_vehicles_reach_their_goal_region(
    ego_vehicles: Dict[int, DynamicObstacle],
    planning_problem_set: PlanningProblemSet,
    time_step: int,
) -> bool:
    return all(
        _did_ego_vehicle_reach_goal_region(
            ego_vehicles[planning_problem.planning_problem_id],
            planning_problem,
            time_step,
        )
        for planning_problem in planning_problem_set.planning_problem_dict.values()
    )


def _did_ego_vehicle_reach_goal_region(
    ego_vehicle: DynamicObstacle, planning_problem: PlanningProblem, time_step: int
) -> bool:
    if ego_vehicle.prediction is None:
        return False
    assert isinstance(ego_vehicle.prediction, TrajectoryPrediction)

    ego_state_at_time_step = ego_vehicle.state_at_time(time_step)
    if ego_state_at_time_step is None:
        return False

    goal_reached = planning_problem.goal.is_reached(ego_state_at_time_step)
    return goal_reached


def _reduce_scenario_to_interactive_scenario(scenario: Scenario) -> Scenario:
    """
    Create a new CommonRoad scenario with the same metadata and lanelet network as :param:`scenario`, but without the trajectories of the dynamic obstacles.

    :param scenario: The scenario to reduce
    :returns: The reduced, interactive scenario
    """
    new_scenario = create_new_scenario_with_metadata_from_old_scenario(scenario)

    # Only add a reference of the lanelet network to the scenario
    new_scenario.add_objects(scenario.lanelet_network)

    for original_obstacle in scenario.dynamic_obstacles:
        new_obstacle = DynamicObstacle(
            original_obstacle.obstacle_id,
            original_obstacle.obstacle_type,
            obstacle_shape=original_obstacle.obstacle_shape,
            initial_state=copy.deepcopy(original_obstacle.initial_state),
        )
        new_scenario.add_objects(new_obstacle)

    return new_scenario


def _update_trajectory_prediction(
    dynamic_obstacle: DynamicObstacle, trajectory: Trajectory
):
    """
    Update the trajectory prediction of a dynamic obstacle by the trajectory. If no prediction exists, a new one is created. Otherwise, :param:`trajectory` is appened to the existing trajectory.
    """
    if dynamic_obstacle.prediction is None:
        dynamic_obstacle.prediction = TrajectoryPrediction(
            trajectory=trajectory, shape=dynamic_obstacle.obstacle_shape
        )
    else:
        assert isinstance(dynamic_obstacle.prediction, TrajectoryPrediction)
        for state in trajectory.state_list:
            dynamic_obstacle.prediction.trajectory.append_state(state)


def _cut_trajectory(trajectory: Trajectory, length: int) -> Trajectory:
    """
    Cut the given :param:`trajectory` to :param:`length` states
    """
    return Trajectory(
        initial_time_step=trajectory.initial_time_step,
        state_list=trajectory.state_list[:length],
    )


def _create_id_mapper_for_interactive_scenario(scenario: Scenario) -> IdMapper:
    """
    Create an IdMapper for interactive simulations.

    Pre-conditions the IdMapper with ID mappings for all dynamic obstacles present in the interactive scenario.

    :param scenario: The interactive scenario.

    :returns: The pre-conditioned IdMapper.
    """
    id_mapper = IdMapper.from_scenario(scenario, strict=True)

    for dynamic_obstacle in scenario.dynamic_obstacles:
        cr_id = dynamic_obstacle.obstacle_id
        sumo_id = str(cr_id)
        id_mapper.insert_mapping(sumo_id, cr_id)

    return id_mapper
