__all__ = [
    "SumoSimulationBackendConfiguration",
    "SumoSimulationBackend",
    "LibsumoSumoSimulationBackend",
    "TraciSumoSimulationBackend",
]
import logging
from dataclasses import dataclass
from os import PathLike
from typing import Any, List, Optional, Protocol, Tuple, Type, TypeVar, Union, cast

import libsumo
import traci
from commonroad.scenario.scenario import Scenario
from typing_extensions import Self

from commonroad_sumo.backend.types import (
    Collision,
    SumoAutomaticRouting,
    SumoDriverState,
    SumoGlosa,
    SumoId,
    SumoJunctionModel,
    SumoLaneChangeMode,
    SumoLaneChangeModel,
    SumoParameterCollection,
    SumoSignalState,
    SumoSpeedMode,
    SumoVehicleClass,
)
from commonroad_sumo.errors import SumoInterfaceError, SumoSimulationError
from commonroad_sumo.helpers import get_sumo_binary_path, get_sumo_gui_binary_path

_LOGGER = logging.getLogger(__name__)

_T_parameter_collection = TypeVar(
    "_T_parameter_collection", bound=SumoParameterCollection
)


@dataclass
class SumoSimulationBackendConfiguration:
    dt: float = 0.1
    delta_steps: float = 1.0
    lateral_resolution: float = 1.0
    random_seed: Optional[float] = None
    quit_on_end: bool = False

    collision: bool = True
    """Toggle collisions on/off in the simulation"""

    driver_state: bool = True
    """Toggle driver state device on/off for *all* vehicles in the simulation"""

    glosa: bool = False
    """Toggle glosa device on/off for *all* vehicles in the simulation"""

    automatic_routing: bool = False
    """Toggle reroute device on/off for *all* vehicles in the simulation"""

    @classmethod
    def for_scenario(cls, scenario: Scenario) -> Self:
        return cls(dt=scenario.dt)


# Types for the traci and libsumo domains
# For the traci types, we need to access private modules, because the types are not exposed anywhere
VehicleDomain = Union[traci.main._vehicle.VehicleDomain, libsumo.vehicle]
RouteDomain = Union[traci.main._route.RouteDomain, libsumo.route]
VehicleTypeDomain = Union[
    traci.main._vehicletype.VehicleTypeDomain, libsumo.vehicletype
]
PersonDomain = Union[traci.main._person.PersonDomain, libsumo.person]
SimulationDomain = Union[traci.main._simulation.SimulationDomain, libsumo.simulation]
TrafficLightDomain = Union[
    traci.main._trafficlight.TrafficLightDomain, libsumo.trafficlight
]
LaneDomain = Union[traci.main._lane.LaneDomain, libsumo.lane]
EdgeDomain = Union[traci.main._edge.EdgeDomain, libsumo.edge]


SumoDomain = Union[
    VehicleDomain,
    RouteDomain,
    VehicleTypeDomain,
    PersonDomain,
    SimulationDomain,
    TrafficLightDomain,
    LaneDomain,
    EdgeDomain,
]


class SumoApi(Protocol):
    """
    Interface which models the TraCI interface.

    libsumo and traci already satisfy this protocol.
    """

    # The different domains which must be supported by a TraCI compatible API.
    vehicle: VehicleDomain
    route: RouteDomain
    vehicletype: VehicleTypeDomain
    person: PersonDomain
    simulation: SimulationDomain
    trafficlight: TrafficLightDomain
    lane: LaneDomain
    edge: EdgeDomain

    # The base exception type used by this TraCI compatible API.
    # This is used by the backend to consistently handle exceptions from the API.
    TraCIException: type[Exception]

    @staticmethod
    def start(cmd: List[str]) -> None: ...

    @staticmethod
    def simulationStep() -> None: ...

    @staticmethod
    def close() -> None: ...

    @staticmethod
    def isLoaded() -> bool: ...


class SumoSimulationBackend:
    """
    This is an abstract interface for libraries that support the traci API (libsumo and traci).
    """

    def __init__(
        self,
        config: SumoSimulationBackendConfiguration,
        sumo_api: SumoApi,
        sumo_binary_path: PathLike,
    ):
        """
        :param config: The configuration object for simulation
        :param sumo_config_file: Path to the sumo configuration file
        :param sumo_interface: The library that provides the concrete interactions with sumo
        :param sumo_binary_path: Path to the sumo binary that will be used for execution
        """
        self._config = config
        self._sumo_api = sumo_api
        self._sumo_binary_path = sumo_binary_path
        self._vehicle_ids = None
        self._person_ids = None

    def initialize(self):
        pass

    def start(self, sumo_config_file: PathLike):
        """
        Configure and start the underlying sumo interface to begin the simulation
        """

        dt_sumo: float = self._config.dt / self._config.delta_steps
        cmd: List[str] = [
            str(self._sumo_binary_path),
            "-c",
            str(sumo_config_file),
            "--step-length",
            str(dt_sumo),
            "--lateral-resolution",
            str(self._config.lateral_resolution),
        ]

        if self._config.lateral_resolution > 0.0:
            cmd.extend(["--lanechange.duration", "0"])

        if self._config.random_seed is not None:
            cmd.extend(["--seed", str(self._config.random_seed)])

        if self._config.quit_on_end:
            cmd.append("--quit-on-end")

        if self._config.collision:
            # Activate collisions. With other settings, we cannot receive information about collisions.
            cmd.extend(["--collision.action", "warn"])
            # If activated, all collisions are considered
            cmd.extend(["--collision.check-junctions", "true"])

        if self._config.driver_state:
            # Set probability to 100% so that every device becomes a driverstate device. This is necessary, because we cannot dynamically enable a driver state device on a vehicle. Therefore, the driver state device is enabled for all devices and can later be configured for each individual vehicle
            cmd.extend(["--device.driverstate.probability", "1"])

        if self._config.automatic_routing:
            # See description for driver state device above
            cmd.extend(["--device.rerouting.probability", "1"])

        if self._config.glosa:
            # See description for driver state device above
            cmd.extend(["--device.glosa.probability", "1"])

        _LOGGER.debug(f"Executing SUMO simulation with '{cmd}'")
        self._sumo_api.start(cmd)

    def _flush(self):
        self._vehicle_ids = None
        self._person_ids = None

    def simulation_step(self):
        """
        Perform a simulation step
        """
        try:
            self._sumo_api.simulationStep()
        except libsumo.libsumo.FatalTraCIError as e:
            self.stop()
            raise SumoSimulationError(
                f"Simulation step failed unexpectly and simulation was shutdown: {e}"
            ) from e

        self._flush()

    def stop(self):
        """
        Stop the running simulation and close the connection to sumo
        """
        self._sumo_api.close()

    def _get_vehicle_resource(
        self, vehicle_id: SumoId, resource: int, fallback_fn
    ) -> Any:
        return fallback_fn(vehicle_id)

    @property
    def _route_domain(self) -> RouteDomain:
        return self._sumo_api.route

    @property
    def _vehicle_domain(self) -> VehicleDomain:
        return self._sumo_api.vehicle

    @property
    def _vehicle_type_domain(self) -> VehicleTypeDomain:
        return self._sumo_api.vehicletype

    @property
    def _person_domain(self) -> PersonDomain:
        return self._sumo_api.person

    @property
    def _simulation_domain(self) -> SimulationDomain:
        return self._sumo_api.simulation

    @property
    def _traffic_light_domain(self) -> TrafficLightDomain:
        return self._sumo_api.trafficlight

    @property
    def _lane_domain(self) -> LaneDomain:
        return self._sumo_api.lane

    @property
    def _edge_domain(self) -> EdgeDomain:
        return self._sumo_api.edge

    def _set_parameter_collection(
        self,
        domain: SumoDomain,
        resource_id: SumoId,
        parameters: SumoParameterCollection,
    ) -> None:
        """
        Helper method to apply all values from a parameter collection on the vehicle.

        :param domain: Specify the SUMO domain for the resource
        :param vehicle_id: The ID of the resource, for which the parameters are configured
        :param parameters: The parameter collection
        """
        for (
            parameter_key,
            parameter_value,
        ) in parameters.to_sumo_parameter_map().items():
            domain.setParameter(resource_id, parameter_key, str(parameter_value))

    def _get_parameter_collection(
        self,
        domain: SumoDomain,
        resource_id: SumoId,
        parameter_collection_type: Type[_T_parameter_collection],
    ) -> _T_parameter_collection:
        """
        Helper method to query all values for a parameter collection for a vehicle.

        :param domain: Specify the SUMO domain for the resource
        :param vehicle_id: The ID of the resource, for which the parameters are queried
        :param parameter_collection_type: The type of the parameter collection
        """
        parameter_map = dict()
        for parameter_key in parameter_collection_type.get_parameter_keys():
            parameter_value = domain.getParameter(resource_id, parameter_key)
            parameter_map[parameter_key] = parameter_value

        # Work around mypy bug https://github.com/python/mypy/issues/10003
        parameter_collection: _T_parameter_collection = cast(
            _T_parameter_collection,
            parameter_collection_type.from_sumo_parameter_map(parameter_map),
        )
        return parameter_collection

    def get_vehicle_ids(self) -> Tuple[SumoId, ...]:
        if self._vehicle_ids is None:
            self._vehicle_ids = self._vehicle_domain.getIDList()
            assert self._vehicle_ids is not None  # make mypy happy
        return self._vehicle_ids

    def add_vehicle(
        self, vehicle_id: SumoId, vehicle_class: Union[str, SumoVehicleClass]
    ):
        """
        Adds a new vehicle to the SUMO simulation and adjusts its vehicle class and shape accordingly.

        :param vehicle_id: An unoccupied ID which will be assigned to the new vehicle.
        :param vehicle_class: Class for the new vehicle.

        :returns: Nothing.

        :raises SumoInterfaceError: If the vehicle could not be added.
        """
        try:
            self._vehicle_domain.add(vehicle_id, "")  # routeId is empty
        except self._sumo_api.TraCIException as exc:
            raise SumoInterfaceError(
                f"Failed to add vehicle {vehicle_id} to SUMO simulation: {exc}"
            )

        if isinstance(vehicle_class, SumoVehicleClass):
            self._vehicle_domain.setVehicleClass(vehicle_id, vehicle_class.value)
            self._vehicle_domain.setShapeClass(vehicle_id, vehicle_class.value)
        else:
            self._vehicle_domain.setVehicleClass(vehicle_id, vehicle_class)
            self._vehicle_domain.setShapeClass(vehicle_id, vehicle_class)

    def move_vehicle(self, vehicle_id: SumoId, x: float, y: float, angle: float):
        """
        Moves a vehicle to the given cartesian coordinates.

        This does not perform any route checks, so a vehicle can also be moved outside the road network.
        The caller must ensure that the movement is correct.

        :param vehicle_id: ID of vehicle that should be moved.
        :param x: Cartesian x coordinate to which the vehicle should be moved.
        :param y: Cartesian y coordinate to which the vehicle should be moved.
        :param angle: The orientation of the vehicle in the end.

        :returns: Nothing.

        :raises SumoInterfaceError: If the vehicle cannot be moved. The most common reason is that the vehicle does not exist in the simulation.
        """
        try:
            self._vehicle_domain.moveToXY(
                vehicle_id,
                # edgeID is set to 'dummy' as it is only considered a placement hint.
                # TODO: For complex scenarios it might make sense to use our lanelet -> edge mapping to ensure a more consistent syncing.
                "dummy",
                # Since edgeID is not used, laneIndex is also irrelevant.
                -1,
                x,
                y,
                angle,
                # Move to any edge in the route network. This also allows the vehicle to move out of its original route.
                keepRoute=2,
            )
        except self._sumo_api.TraCIException as exc:
            raise SumoInterfaceError(
                f"Failed to move SUMO vehicle {vehicle_id} to ({x},{y}): {exc}"
            )

    def remove_vehicle(self, vehicle_id: SumoId):
        self._vehicle_domain.remove(vehicle_id)

    def set_vehicle_shape(self, vehicle_id: SumoId, length: float, width: float):
        # TODO: match shape input argument with get_vehicle_shape
        self._vehicle_domain.setLength(vehicle_id, length)
        self._vehicle_domain.setWidth(vehicle_id, width)

    def set_vehicle_speed(self, vehicle_id: SumoId, speed: float):
        self._vehicle_domain.setSpeed(vehicle_id, speed)

    def set_vehicle_speed_mode(self, vehicle_id: SumoId, speed_mode: SumoSpeedMode):
        speed_mode_bitmask = speed_mode.to_sumo_bitset()
        self._vehicle_domain.setSpeedMode(vehicle_id, speed_mode_bitmask)

    def get_vehicle_speed_mode(self, vehicle_id: SumoId) -> SumoSpeedMode:
        speed_mode_bitmask = self._vehicle_domain.getSpeedMode(vehicle_id)
        return SumoSpeedMode.from_sumo_bitset(speed_mode_bitmask)

    def set_vehicle_acceleration(self, vehicle_id: SumoId, acceleration: float):
        self._vehicle_domain.setAccel(vehicle_id, acceleration)

    def set_vehicle_deceleration(self, vehicle_id: SumoId, deceleration: float):
        self._vehicle_domain.setDecel(vehicle_id, deceleration)

    def set_vehicle_min_gap(self, vehicle_id: SumoId, min_gap: float):
        self._vehicle_domain.setMinGap(vehicle_id, min_gap)

    def set_vehicle_max_speed(self, vehicle_id: SumoId, max_speed: float):
        self._vehicle_domain.setMaxSpeed(vehicle_id, max_speed)

    def get_vehicle_max_speed(self, vehicle_id: SumoId) -> float:
        return self._vehicle_domain.getMaxSpeed(vehicle_id)

    def set_vehicle_tau(self, vehicle_id: SumoId, tau: float):
        self._vehicle_domain.setTau(vehicle_id, tau)

    def get_vehicle_class(self, vehicle_id: SumoId) -> SumoVehicleClass:
        vehicle_class_str: str = self._vehicle_domain.getVehicleClass(vehicle_id)
        return SumoVehicleClass.from_sumo_str(vehicle_class_str)

    def get_vehicle_type(self, vehicle_id: SumoId) -> SumoId:
        return self._vehicle_domain.getTypeID(vehicle_id)

    def get_vehicle_shape(self, vehicle_id: SumoId) -> Tuple[float, float]:
        length: float = self._vehicle_domain.getLength(vehicle_id)
        width: float = self._vehicle_domain.getWidth(vehicle_id)
        return (length, width)

    def get_vehicle_position(self, vehicle_id: SumoId) -> Tuple[float, float]:
        return self._get_vehicle_resource(
            vehicle_id, libsumo.constants.VAR_POSITION, self._vehicle_domain.getPosition
        )

    def get_vehicle_speed(self, vehicle_id: SumoId) -> float:
        return self._get_vehicle_resource(
            vehicle_id, libsumo.constants.VAR_SPEED, self._vehicle_domain.getSpeed
        )

    def get_vehicle_lateral_speed(self, vehicle_id: SumoId) -> float:
        return self._get_vehicle_resource(
            vehicle_id,
            libsumo.constants.VAR_SPEED_LAT,
            self._vehicle_domain.getLateralSpeed,
        )

    def get_vehicle_acceleration(self, vehicle_id: SumoId) -> float:
        return self._get_vehicle_resource(
            vehicle_id,
            libsumo.constants.VAR_ACCELERATION,
            self._vehicle_domain.getAcceleration,
        )

    def get_vehicle_angle(self, vehicle_id: SumoId) -> float:
        return self._get_vehicle_resource(
            vehicle_id, libsumo.constants.VAR_ANGLE, self._vehicle_domain.getAngle
        )

    def get_vehicle_signals(self, vehicle_id: SumoId) -> SumoSignalState:
        signal_bitset = self._get_vehicle_resource(
            vehicle_id, libsumo.constants.VAR_SIGNALS, self._vehicle_domain.getSignals
        )
        return SumoSignalState.from_sumo_bitset(signal_bitset)

    def set_vehicle_signals(self, vehicle_id: SumoId, signal_state: SumoSignalState):
        signal_bitset = signal_state.to_sumo_bitset()
        return self._vehicle_domain.setSignals(vehicle_id, signal_bitset)

    def get_vehicle_lane_change_mode(self, vehicle_id: SumoId) -> SumoLaneChangeMode:
        lane_change_mode_bitset = self._vehicle_domain.getLaneChangeMode(vehicle_id)
        return SumoLaneChangeMode.from_sumo_bitset(lane_change_mode_bitset)

    def set_vehicle_lane_change_mode(
        self, vehicle_id: SumoId, lane_change_mode: SumoLaneChangeMode
    ) -> None:
        lane_change_mode_bitset = lane_change_mode.to_sumo_bitset()
        self._vehicle_domain.setLaneChangeMode(vehicle_id, lane_change_mode_bitset)

    def has_vehicle_driver_state(self, vehicle_id: SumoId) -> bool:
        return bool(
            self._vehicle_domain.getParameter(vehicle_id, "has.driverstate.device")
        )

    def get_vehicle_driver_state(self, vehicle_id: SumoId) -> Optional[SumoDriverState]:
        if not self.has_vehicle_driver_state(vehicle_id):
            return None
        return self._get_parameter_collection(
            self._vehicle_domain, vehicle_id, SumoDriverState
        )

    def set_vehicle_driver_state(
        self, vehicle_id: SumoId, driver_state: SumoDriverState
    ) -> None:
        if not self.has_vehicle_driver_state(vehicle_id):
            raise ValueError(
                f"Tried to set driver state for vehicle '{vehicle_id}', but the driver state device is not available on the vehicle!"
            )
        self._set_parameter_collection(self._vehicle_domain, vehicle_id, driver_state)

    def get_vehicle_lane_change_model(self, vehicle_id: SumoId) -> SumoLaneChangeModel:
        return self._get_parameter_collection(
            self._vehicle_domain, vehicle_id, SumoLaneChangeModel
        )

    def set_vehicle_lane_change_model(
        self, vehicle_id: SumoId, lane_change_model: SumoLaneChangeModel
    ) -> None:
        self._set_parameter_collection(
            self._vehicle_domain, vehicle_id, lane_change_model
        )

    def get_vehicle_automatic_routing(self, vehicle_id: SumoId) -> SumoAutomaticRouting:
        return self._get_parameter_collection(
            self._vehicle_domain, vehicle_id, SumoAutomaticRouting
        )

    def set_vehicle_automatic_routing(
        self, vehicle_id: SumoId, reroute_config: SumoAutomaticRouting
    ) -> None:
        self._set_parameter_collection(self._vehicle_domain, vehicle_id, reroute_config)

    def get_vehicle_type_junction_model(
        self, vehicle_type: SumoId
    ) -> SumoJunctionModel:
        return self._get_parameter_collection(
            self._vehicle_type_domain, vehicle_type, SumoJunctionModel
        )

    def set_vehicle_type_junction_model(
        self, vehicle_type: SumoId, junction_model: SumoJunctionModel
    ) -> None:
        self._set_parameter_collection(
            self._vehicle_type_domain, vehicle_type, junction_model
        )

    def set_vehicle_type_glosa(
        self, vehicle_id: SumoId, glosa_configuration: SumoGlosa
    ) -> None:
        self._set_parameter_collection(
            self._vehicle_type_domain, vehicle_id, glosa_configuration
        )

    def get_vehicle_type_glosa(self, vehicle_id: SumoId) -> SumoGlosa:
        return self._get_parameter_collection(
            self._vehicle_type_domain, vehicle_id, SumoGlosa
        )

    def get_collisions(self) -> Tuple[Collision, ...]:
        collisions = self._simulation_domain.getCollisions()
        # Use tuple to stay consistent with all the other traci APIs which also return tuples, instead of lists
        return tuple(
            [Collision.from_sumo_collision(collision) for collision in collisions]
        )

    def get_person_ids(self) -> Tuple[SumoId, ...]:
        if self._person_ids is None:
            self._person_ids = self._person_domain.getIDList()
            assert self._person_ids is not None  # make mypy happy
        return self._person_ids

    def add_person(self, person_id: SumoId, edgeId: SumoId):
        self._person_domain.add(person_id, edgeId, 0)

    def move_person(self, person_id: SumoId, x: float, y: float, angle: float):
        self._person_domain.moveToXY(person_id, "", x, y, angle, keepRoute=0)

    def remove_person(self, person_id: SumoId):
        self._person_domain.remove(person_id)

    def get_person_position(self, person_id: SumoId) -> Tuple[float, float]:
        position: Tuple[float, float] = self._person_domain.getPosition(person_id)
        return position

    def get_person_speed(self, person_id: SumoId) -> float:
        return self._person_domain.getSpeed(person_id)

    def get_person_angle(self, person_id: SumoId) -> float:
        return self._person_domain.getAngle(person_id)

    def get_person_acceleration(self, person_id: SumoId) -> float:
        acceleration: float = self._person_domain.getAccel(person_id)
        return acceleration

    def set_person_shape(self, person_id: SumoId, length: float):
        self._person_domain.setLength(person_id, length)

    def set_person_min_gap(self, person_id: SumoId, min_gap: float):
        self._person_domain.setMinGap(person_id, min_gap)

    def set_person_speed(self, person_id: SumoId, speed: float) -> None:
        self._person_domain.setSpeed(person_id, speed)

    def get_person_shape(self, person_id: SumoId) -> float:
        length: float = self._person_domain.getLength(person_id)
        return length

    def get_traffic_light_ids(self) -> Tuple[SumoId, ...]:
        return self._traffic_light_domain.getIDList()

    def set_traffic_light_state(self, tls_id: SumoId, state_string: str):
        self._traffic_light_domain.setRedYellowGreenState(tls_id, state_string)

    def get_traffic_light_state(self, tls_id: SumoId) -> str:
        return self._traffic_light_domain.getRedYellowGreenState(tls_id)

    def get_traffic_light_controlled_links(
        self, tls_id: SumoId
    ) -> List[List[Tuple[SumoId, SumoId, SumoId]]]:
        return self._traffic_light_domain.getControlledLinks(tls_id)

    def get_lane_ids(self) -> Tuple[SumoId, ...]:
        return self._lane_domain.getIDList()

    def get_lane_allowed(self, lane_id: SumoId) -> Tuple[SumoId, ...]:
        return self._lane_domain.getAllowed(lane_id)

    def get_edge_id_of_lane(self, lane_id: SumoId) -> SumoId:
        return self._lane_domain.getEdgeID(lane_id)


class TraciSumoSimulationBackend(SumoSimulationBackend):
    """
    Provides an interface to perform sumo simulation with a GUI by utilizing traci.
    """

    def __init__(
        self,
        config: Optional[SumoSimulationBackendConfiguration] = None,
        sumo_binary_path: Optional[PathLike] = None,
    ):
        """
        :param config: The configuration object for simulation
        :param sumo_binary_path: Path to the sumo binary that will be used for execution
        """
        super().__init__(
            config if config is not None else SumoSimulationBackendConfiguration(),
            sumo_api=traci,
            sumo_binary_path=(
                sumo_binary_path
                if sumo_binary_path is not None
                else get_sumo_gui_binary_path()
            ),
        )


class LibsumoSumoSimulationBackend(SumoSimulationBackend):
    """
    Provides an interface to perform sumo simulation without a GUI by utilizing libsumo.
    """

    def __init__(
        self,
        config: Optional[SumoSimulationBackendConfiguration] = None,
        sumo_binary_path: Optional[PathLike] = None,
    ):
        """
        :param config: The configuration object for simulation
        :param sumo_binary_path: Path to the sumo binary that will be used for execution
        """
        super().__init__(
            config if config is not None else SumoSimulationBackendConfiguration(),
            sumo_api=libsumo,
            sumo_binary_path=(
                sumo_binary_path
                if sumo_binary_path is not None
                else get_sumo_binary_path()
            ),
        )
