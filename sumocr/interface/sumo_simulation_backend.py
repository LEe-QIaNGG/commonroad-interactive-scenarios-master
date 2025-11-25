import random
from abc import ABC, abstractmethod
from dataclasses import dataclass
from os import PathLike
from typing import List, Optional, Union

import libsumo
import numpy as np
import traci

from sumocr.helpers import get_sumo_binary_path, get_sumo_gui_binary_path


# TODO: Use a seperate configuration, to decouple this from the DefaultConfig object, that is used all over the place
@dataclass
class SumoSimulationBackendConfiguration:
    dt: int
    delta_steps: int
    lateral_resolution: float
    random_seed: Optional[float] = None


# Types for the traci and libsumo domains
VehicleDomain = Union[traci._vehicle.VehicleDomain, libsumo.vehicle]
RouteDomain = Union[traci._route.RouteDomain, libsumo.route]
VehicleTypeDomain = Union[traci._vehicletype.VehicleTypeDomain, libsumo.vehicletype]
PersonDomain = Union[traci._person.PersonDomain, libsumo.person]
SimulationDomain = Union[traci._simulation.SimulationDomain, libsumo.simulation]
TrafficLightDomain = Union[traci._trafficlight.TrafficLightDomain, libsumo.trafficlight]
LaneDomain = Union[traci._lane.LaneDomain, libsumo.lane]
EdgeDomain = Union[traci._edge.EdgeDomain, libsumo.edge]


class SumoSimulationBackend(ABC):
    """
    SumoSimulationBackend provides an interface for interacting with a more concrete sumo simulator (e.g. based on libsumo).
    """

    def __init__(
        self, config: SumoSimulationBackendConfiguration, sumo_config_file: PathLike
    ):
        """
        :param config: The configuration object for simulation
        :param sumo_config_file: Path to the sumo configuration file
        """
        self._config = config
        self._sumo_config_file = sumo_config_file

    @abstractmethod
    def initialize(self):
        ...

    @abstractmethod
    def start(self):
        ...

    @abstractmethod
    def simulation_step(self):
        ...

    @abstractmethod
    def stop(self):
        ...

    # TODO: the domain handling represents a tight coupling with the expected backend (aka. being traci API compatible).
    # The better approach would probably be, to expose a consistent API Interface here, and rely on the concret backends to implement this.
    # Nevertheless, this is currently not possible, due to the high coupling with the SumoSimulation
    @property
    @abstractmethod
    def route_domain(self) -> RouteDomain:
        ...

    @property
    @abstractmethod
    def vehicle_domain(self) -> VehicleDomain:
        ...

    @property
    @abstractmethod
    def vehicle_type_domain(self) -> VehicleTypeDomain:
        ...

    @property
    @abstractmethod
    def person_domain(self) -> PersonDomain:
        ...

    @property
    @abstractmethod
    def simulation_domain(self) -> SimulationDomain:
        ...

    @property
    @abstractmethod
    def traffic_light_domain(self) -> TrafficLightDomain:
        ...

    @property
    @abstractmethod
    def lane_domain(self) -> LaneDomain:
        ...

    @property
    @abstractmethod
    def edge_domain(self) -> EdgeDomain:
        ...

    # TODO: see above; high coupling with traci API
    def add_vehicle(self, vehicle_id: str, vehicle_class: str):
        self.vehicle_domain.add(vehicle_id, "")  # routeId is empty
        self.vehicle_domain.setVehicleClass(vehicle_id, vehicle_class)
        self.vehicle_domain.setShapeClass(vehicle_id, vehicle_class)

    def move_vehicle(self, vehicle_id: str, x: float, y: float, angle: float):
        self.vehicle_domain.moveToXY(vehicle_id, "dummy", -1, x, y, angle)

    def remove_vehicle(self, vehicle_id: str):
        self.vehicle_domain.remove(vehicle_id)

    def set_vehicle_shape(self, vehicle_id: str, length: float, width: float):
        traci.vehicle.setLength(vehicle_id, length)
        traci.vehicle.setWidth(vehicle_id, width)

    def add_person(self, person_id: str, edgeId: str):
        self.person_domain.add(person_id, edgeId, 0)

    def move_person(self, person_id: str, x: float, y: float, angle: float):
        self.person_domain.moveToXY(person_id, "", x, y, angle, keepRoute=0)

    def remove_person(self, person_id: str):
        self.person_domain.remove(person_id)


class TraciApiSumoSimulationBackend(SumoSimulationBackend):
    """
    This is an abstract interface for libraries that support the traci API (libsumo and traci).
    """

    def __init__(
        self,
        config: SumoSimulationBackendConfiguration,
        sumo_config_file: PathLike,
        sumo_interface: Union["libsumo", "traci"],
        sumo_binary_path: PathLike,
    ):
        """
        :param config: The configuration object for simulation
        :param sumo_config_file: Path to the sumo configuration file
        :param sumo_interface: The library that provides the concrete interactions with sumo
        :param sumo_binary_path: Path to the sumo binary that will be used for execution
        """
        super().__init__(config, sumo_config_file)
        self._sumo_interface = sumo_interface
        self._sumo_binary_path = sumo_binary_path

    def initialize(self):
        pass

    def start(self):
        """
        Configure and start the underlying sumo interface to begin the simulation
        """
        dt_sumo: float = self._config.dt / self._config.delta_steps
        cmd: List[str] = [
            str(self._sumo_binary_path),
            "-c",
            str(self._sumo_config_file),
            "--step-length",
            str(dt_sumo),
            "--lateral-resolution",
            str(self._config.lateral_resolution),
            # TODO: should --quit-on-end be configurable by the caller?
            # "--quit-on-end",
        ]

        if self._config.lateral_resolution > 0.0:
            cmd.extend(["--lanechange.duration", "0"])

        if self._config.random_seed:
            # REFACTOR: Is this seeding needed and is this the right place for it?
            np.random.seed(self._config.random_seed)
            random.seed(self._config.random_seed)
            cmd.extend(["--seed", str(self._config.random_seed)])

        self._sumo_interface.start(cmd)

    def simulation_step(self):
        """
        Perform a simulation step
        """
        self._sumo_interface.simulationStep()

    def stop(self):
        """
        Stop the running simulation and close the connection to sumo
        """
        self._sumo_interface.close()

    @property
    def route_domain(self):
        return self._sumo_interface.route

    @property
    def vehicle_domain(self):
        return self._sumo_interface.vehicle

    @property
    def vehicle_type_domain(self):
        return self._sumo_interface.vehicletype

    @property
    def person_domain(self):
        return self._sumo_interface.person

    @property
    def simulation_domain(self):
        return self._sumo_interface.simulation

    @property
    def traffic_light_domain(self):
        return self._sumo_interface.trafficlight

    @property
    def lane_domain(self):
        return self._sumo_interface.lane

    @property
    def edge_domain(self):
        return self._sumo_interface.edge


class GuiSumoSimulationBackend(TraciApiSumoSimulationBackend):
    """
    Provides an interface to perform sumo simulation with a GUI by utilizing traci.
    """

    def __init__(
        self,
        config: SumoSimulationBackendConfiguration,
        sumo_config_file: PathLike,
        sumo_binary_path: Optional[PathLike] = None,
    ):
        """
        :param config: The configuration object for simulation
        :param sumo_config_file: Path to the sumo configuration file
        :param sumo_binary_path: Path to the sumo binary that will be used for execution
        """
        super().__init__(
            config,
            sumo_config_file,
            sumo_interface=traci,
            sumo_binary_path=sumo_binary_path
            if sumo_binary_path is not None
            else get_sumo_gui_binary_path(),
        )


class HeadlessSumoSimulationBackend(TraciApiSumoSimulationBackend):
    """
    Provides an interface to perform sumo simulation without a GUI by utilizing libsumo.
    """

    def __init__(
        self,
        config: SumoSimulationBackendConfiguration,
        sumo_config_file: PathLike,
        sumo_binary_path: Optional[PathLike] = None,
    ):
        """
        :param config: The configuration object for simulation
        :param sumo_config_file: Path to the sumo configuration file
        :param sumo_binary_path: Path to the sumo binary that will be used for execution
        """
        # TODO: add a check to verify that the binary is not a sumo-gui binary
        super().__init__(
            config,
            sumo_config_file,
            sumo_interface=libsumo,
            sumo_binary_path=sumo_binary_path
            if sumo_binary_path is not None
            else get_sumo_binary_path(),
        )
