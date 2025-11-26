import logging
from pathlib import Path
from typing import Set, Union

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.scenario import Scenario
from typing_extensions import Self, Unpack, override

from commonroad_sumo.cr2sumo import (
    AbstractTrafficGenerator,
    CR2SumoMapConverter,
    SumoTrafficGenerationMode,
    create_traffic_generator_for_mode,
)
from commonroad_sumo.errors import SumoInterfaceError
from commonroad_sumo.helpers import create_new_scenario_with_metadata_from_old_scenario
from commonroad_sumo.simulation.abstract_sumo_simulation import (
    AbstractSumoSimulation,
    SumoSimulationExtraArguments,
    SumoSimulationResult,
)
from commonroad_sumo.sumolib.sumo_project import SumoProject

_LOGGER = logging.getLogger(__name__)


class NonInteractiveSumoSimulation(AbstractSumoSimulation):
    """
    A Non-Interactive SUMO simulation is used to simulate a CommonRoad scenario without any involvment in the simulation loop.
    During the simulation all obstacles will be synchronized from SUMO to CommonRoad.
    """

    def __init__(
        self,
        scenario: Scenario,
        sumo_project: SumoProject,
        **kwargs: Unpack[SumoSimulationExtraArguments],
    ) -> None:
        # A new scenario object is created, which serves two purposes:
        # 1. New objects will be synced into this scenario, but the caller might want to keep using their scenario without the modifications from the simulation.
        # 2. The `SumoSimulation` will create an `IdMapper` with `ExhaustiveIdAllocator`. To make
        #    sure, that ID preservation works, a new scenario is needed, where the dynamic obstacle IDs are missing.
        #    Otherwise, the `ExhaustiveIdAllocator` would mark them as already used.
        new_scenario = _create_new_scenario_for_non_interactive_simulation(scenario)
        super().__init__(new_scenario, sumo_project, **kwargs)

        # Keep track of active obstacles, which need to be synced.
        # To increase the simulation performance, only their ids are tracked in a set.
        # If the obstacles themselves would be kept in a set,
        # they would need to be hased multiple times during the simulation loop.
        # This is costly and introduces a massive overhead!
        self._active_obstacles: Set[int] = set()

        self._dirty = False
        self._simulation_steps = 0

    @classmethod
    def from_scenario(
        cls,
        scenario: Scenario,
        traffic_generator_or_mode: Union[
            AbstractTrafficGenerator, SumoTrafficGenerationMode
        ] = SumoTrafficGenerationMode.RANDOM,
        **kwargs: Unpack[SumoSimulationExtraArguments],
    ) -> Self:
        converter = CR2SumoMapConverter(scenario)
        sumo_project = converter.create_sumo_files()

        if sumo_project is None:
            raise SumoInterfaceError(
                f"Cannot initialize non-interactive SUMO simulation for scenario {scenario.scenario_id}: The SUMO conversion failed!"
            )

        if not isinstance(traffic_generator_or_mode, AbstractTrafficGenerator):
            traffic_generator_or_mode = create_traffic_generator_for_mode(
                traffic_generator_or_mode
            )
        traffic_generation_sucessful = traffic_generator_or_mode.generate_traffic(
            scenario, sumo_project
        )
        if not traffic_generation_sucessful:
            raise SumoInterfaceError(
                f"Failed to initialize non-interactive SUMO simulation for scenario {scenario.scenario_id}: The traffic generation failed due to an unknown reason"
            )

        return cls(scenario, sumo_project, **kwargs)

    @classmethod
    def from_file(
        cls,
        file_path: Union[str, Path],
        traffic_generator_or_mode: Union[
            AbstractTrafficGenerator, SumoTrafficGenerationMode
        ] = SumoTrafficGenerationMode.RANDOM,
        **kwargs: Unpack[SumoSimulationExtraArguments],
    ) -> Self:
        scenario, _ = CommonRoadFileReader(file_path).open()
        return cls.from_scenario(scenario, traffic_generator_or_mode, **kwargs)

    @override
    def _reached_simulation_end_condition(self, time_step: int) -> bool:
        return time_step == self._simulation_steps

    @override
    def _pre_simulation_step_hook(self, time_step: int) -> None:
        newly_inactive_obstacles = []
        for active_dynamic_obstacle_id in self._active_obstacles:
            dynamic_obstacle = self._scenario.obstacle_by_id(active_dynamic_obstacle_id)
            if not isinstance(dynamic_obstacle, DynamicObstacle):
                raise SumoInterfaceError(
                    f"Excepted obstacle {active_dynamic_obstacle_id} to be a `DynamicObstacle`, but found a `{type(dynamic_obstacle)}`"
                )
            synced = self.sync_from_sumo_simulation(dynamic_obstacle)
            if not synced:
                # The obstacle has left the simulation. It must not be synced anymore.
                newly_inactive_obstacles.append(active_dynamic_obstacle_id)

        if len(newly_inactive_obstacles) > 0:
            self._active_obstacles.symmetric_difference_update(newly_inactive_obstacles)

        new_obstacles = self.fetch_new_from_sumo_simulation()
        for new_dynamic_obstacle in new_obstacles:
            self._scenario.add_objects(new_dynamic_obstacle)
            self._active_obstacles.add(new_dynamic_obstacle.obstacle_id)

        return

    def run(self, simulation_steps: int) -> SumoSimulationResult:
        """
        Run the Non-Interactive simulation and export it as a scenario.
        """
        if self._dirty:
            raise SumoInterfaceError("")

        self._dirty = True
        self._simulation_steps = simulation_steps

        self.simulation_loop()

        return SumoSimulationResult(self._scenario)


def _create_new_scenario_for_non_interactive_simulation(scenario: Scenario) -> Scenario:
    """
    Create a new scenario from an old scenario and include all its metadata.

    :param scenario: The old scenario, from which the metadata will be taken

    :returns: The new scenario with all metadata, which is safe to modify.
    """

    new_scenario = create_new_scenario_with_metadata_from_old_scenario(scenario)

    new_lanelet_network = LaneletNetwork.create_from_lanelet_network(
        scenario.lanelet_network
    )
    new_scenario.add_objects(new_lanelet_network)
    return new_scenario
