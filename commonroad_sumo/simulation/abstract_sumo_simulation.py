import logging
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, TypedDict, Union

from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.scenario import Scenario, TrafficLight

from commonroad_sumo.backend.sumo_simulation_backend import (
    LibsumoSumoSimulationBackend,
    SumoSimulationBackend,
    SumoSimulationBackendConfiguration,
)
from commonroad_sumo.interface.driving_model_parameters_provider import (
    DrivingModelParametersProvider,
)
from commonroad_sumo.interface.id_mapper import IdMapper
from commonroad_sumo.interface.sumo_simulation_interface import SumoSimulationInterface
from commonroad_sumo.sumolib.sumo_project import SumoFileType, SumoProject

_LOGGER = logging.getLogger(__name__)


@dataclass
class SumoSimulationResult:
    """
    The result of a SUMO simulation.
    """

    scenario: Scenario

    def write_to_file(self, file_path: Union[str, Path]) -> None:
        if isinstance(file_path, Path):
            file_path = str(file_path.absolute())
        CommonRoadFileWriter(self.scenario, PlanningProblemSet()).write_to_file(
            file_path, overwrite_existing_file=OverwriteExistingFile.ALWAYS
        )

    def write_to_folder(self, folder_path: Union[str, Path]) -> None:
        if isinstance(folder_path, str):
            folder_path = Path(folder_path)

        file_path = folder_path / f"{self.scenario.scenario_id}.xml"

        self.write_to_file(file_path)


@dataclass
class SumoSimulationConfig:
    lateral_resolution: float = 1.0
    random_seed: int = 1234


class SumoSimulationExtraArguments(TypedDict, total=False):
    simulation_config: Optional[SumoSimulationConfig]
    simulation_backend: Optional[SumoSimulationBackend]
    driving_model_parameters_provider: Optional[DrivingModelParametersProvider]
    id_mapper: Optional[IdMapper]


class AbstractSumoSimulation(ABC):
    """
    A generic base class for simulations in SUMO. This class should be subclassed
    to define specific types of simulations with custom behavior.

    :param scenario: The CommonRoad scenario that should be simulated
    :param sumo_project: A wrapper around an SUMO project which matches the `scenario`.
    :param simulation_backend: Provide an optional existing and configured backend for the simulation. If None is provided, a default
    :param id_mapper: Provide an initialized `IdMapper`. This is useful to make sure that specific CommonRoad IDs won't be overridden.
    """

    def __init__(
        self,
        scenario: Scenario,
        sumo_project: SumoProject,
        simulation_config: Optional[SumoSimulationConfig] = None,
        simulation_backend: Optional[SumoSimulationBackend] = None,
        driving_model_parameters_provider: Optional[
            DrivingModelParametersProvider
        ] = None,
        id_mapper: Optional[IdMapper] = None,
    ) -> None:
        self._scenario = scenario
        self._sumo_project = sumo_project
        if simulation_config is None:
            self._sumo_simulation_config = SumoSimulationConfig()
        else:
            self._sumo_simulation_config = simulation_config

        if simulation_backend is not None:
            self._simulation_backend = simulation_backend
        else:
            simulation_backend_config = SumoSimulationBackendConfiguration(
                dt=scenario.dt,
                lateral_resolution=self._sumo_simulation_config.lateral_resolution,
                random_seed=self._sumo_simulation_config.random_seed,
            )
            self._simulation_backend = LibsumoSumoSimulationBackend(
                simulation_backend_config
            )

        if id_mapper is None:
            self._id_mapper = IdMapper.from_scenario(self._scenario)
        else:
            self._id_mapper = id_mapper

        self._driving_model_parameters_provider = driving_model_parameters_provider

        self._sumo_interface = SumoSimulationInterface(
            self._simulation_backend,
            self._id_mapper,
            self._scenario,
            self._driving_model_parameters_provider,
        )

    def fetch_new_from_sumo_simulation(
        self,
    ) -> List[DynamicObstacle]:
        """
        Fetches new obstacles from SUMO, that have entered the simulation since the last time step.

        :returns: A list of new dynamic obstacles, that have entered the simulation.
                  The initial state of the dynamic obstacles is already populated
                  with the data of the current time step.
        """
        new_obstacles = self._sumo_interface.fetch_new_from_sumo_simulation()
        return new_obstacles

    def sync_to_sumo_simulation(
        self, resource: Union[DynamicObstacle, TrafficLight]
    ) -> bool:
        """
        Syncs a given resource, such as a dynamic obstacle or traffic light, to the SUMO simulation.
        The resource might not yet be present in the simulation, and will be created on demand.

        :param resource: The resource to sync to the SUMO simulation.

        :returns: True if the resource was successfully synced; False otherwise.
        """
        return self._sumo_interface.sync_to_sumo_simulation(resource)

    def sync_from_sumo_simulation(
        self, resource: Union[DynamicObstacle, TrafficLight]
    ) -> bool:
        return self._sumo_interface.sync_from_sumo_simulation(resource)

    @abstractmethod
    def _reached_simulation_end_condition(self, time_step: int) -> bool:
        """
        Abstract method that will be called from `simulation_loop` during each simulation step,
        to determine whether the simulation should continue.
        """
        ...

    def _pre_simulation_step_hook(self, time_step: int) -> None:
        """
        This method will be called during the simulation loop, before the time step is incremented and
        before the simulation will be stepped by one time step.
        """
        return

    def _post_simulation_step_hook(self, time_step: int) -> None:
        """
        This method will be called during the simulation loop, after the time step is incremented and
        after the simulation has been stepped by one time step.
        """
        return

    def simulation_loop(self) -> None:
        start_time = time.time_ns()
        self._simulation_backend.start(
            self._sumo_project.get_file_path(SumoFileType.CONFIG)
        )

        try:
            time_step = 0
            while not self._reached_simulation_end_condition(time_step):
                self._pre_simulation_step_hook(time_step)

                self._sumo_interface.simulate_step()
                time_step += 1

                self._post_simulation_step_hook(time_step)
        finally:
            self._simulation_backend.stop()

        end_time = time.time_ns()
        total_simulation_time_in_s = round((end_time - start_time) / 1000000000, 2)
        _LOGGER.debug(
            "Simulated scenario %s in %ss",
            self._scenario.scenario_id,
            total_simulation_time_in_s,
        )
