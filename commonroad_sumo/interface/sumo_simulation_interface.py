import logging
from abc import ABC
from enum import Enum, auto
from typing import List, Optional, Sequence, Union

from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.traffic_light import TrafficLight

from commonroad_sumo.backend import SumoSimulationBackend
from commonroad_sumo.errors import SumoInterfaceError
from commonroad_sumo.interface.driving_model_parameters_provider import (
    DrivingModelParametersProvider,
)
from commonroad_sumo.interface.id_mapper import CommonRoadId, IdMapper
from commonroad_sumo.interface.interfaces import (
    PedestrianInterface,
    TrafficlightInterface,
    VehicleInterface,
)

_LOGGER = logging.getLogger(__name__)


def _obstacle_is_vehicle(obstacle_type: ObstacleType) -> bool:
    return (
        obstacle_type is ObstacleType.CAR
        or obstacle_type is ObstacleType.TRUCK
        or obstacle_type is ObstacleType.BICYCLE
        or obstacle_type is ObstacleType.BUS
        or obstacle_type is ObstacleType.MOTORCYCLE
    )


def _obstacle_is_pedestrian(obstacle_type: ObstacleType) -> bool:
    return obstacle_type is ObstacleType.PEDESTRIAN


class SimulationEvent(ABC): ...


class CollisionEvent(SimulationEvent):
    class CollisionType(Enum):
        # https://sumo.dlr.de/docs/Simulation/Output/Collisions.html#collision_types
        # rear collision (leader vehicle is the victim)
        REAR = auto()
        # collision while driving through the opposite direction lane (the vehicle using the opposite direction lane is the collider)
        FRONTAL = auto()
        # collision between vehicles on a junction. Collider and Victim are assigned arbitrarily.
        JUNCTION = auto()
        # collision between vehicle and person on the same lane. The person is always the victim
        SHARED_LANE = auto()
        # collision between vehicle and person on a pedestrian crossing. The person is always the victim
        CROSSING = auto()
        # collision between vehicle and person on a walkingarea. The person is always the victim
        WALKINGAREA = auto()
        # other collision between vehicle and person on a junction. The person is always the victim
        JUNCTION_PEDESTRIAN = auto()

        @classmethod
        def from_sumo_type(
            cls, sumo_collision_type: str
        ) -> "CollisionEvent.CollisionType":
            collision_type_mapping = {
                "collision": cls.REAR,
                "frontal": cls.FRONTAL,
                "junction": cls.JUNCTION,
                "sharedLane": cls.SHARED_LANE,
                "crossing": cls.CROSSING,
                "walkingarea": cls.WALKINGAREA,
                "junctionPedestran": cls.JUNCTION_PEDESTRIAN,
            }
            if sumo_collision_type not in collision_type_mapping:
                raise ValueError(
                    f"Invalid SUMO collision type: {sumo_collision_type}. Valid collision types are: {list(collision_type_mapping.keys())}"
                )

            return collision_type_mapping[sumo_collision_type]

    def __init__(
        self,
        collider_id: CommonRoadId,
        victim_id: CommonRoadId,
        collision_type: CollisionType,
    ) -> None:
        self._collider_id = collider_id
        self._victim_id = victim_id
        self._collision_type = collision_type

    @property
    def collider_id(self) -> CommonRoadId:
        return self._collider_id

    @property
    def victim_id(self) -> CommonRoadId:
        return self._victim_id

    @property
    def collision_type(self) -> CollisionType:
        return self._collision_type


class SumoSimulationInterface:
    """
    Provides an interface to the SUMO simulation, by exposing several subinterfaces.
    """

    def __init__(
        self,
        simulation_backend: SumoSimulationBackend,
        id_mapper: IdMapper,
        scenario: Scenario,
        driving_model_parameters_provider: Optional[
            DrivingModelParametersProvider
        ] = None,
    ):
        self._simulation_backend = simulation_backend
        self._id_mapper = id_mapper
        self._scenario = scenario

        self.vehicles = VehicleInterface(
            simulation_backend, id_mapper, driving_model_parameters_provider
        )
        self.pedestrians = PedestrianInterface(simulation_backend, id_mapper)
        self.traffic_lights = TrafficlightInterface(
            simulation_backend, id_mapper, scenario
        )

    def simulate_step(self):
        """
        Perform a simulation step of all interfaces
        """
        self.vehicles.simulate_step()
        self.pedestrians.simulate_step()
        self.traffic_lights.simulate_step()
        self._simulation_backend.simulation_step()

    def sync_traffic_light_to_sumo_simulation(self, traffic_light) -> bool:
        return self.traffic_lights.sync_to_sumo_simulation(traffic_light)

    def sync_traffic_light_from_sumo_simulation(self, traffic_light) -> bool:
        return self.traffic_lights.sync_from_sumo_simulation(traffic_light)

    def sync_obstacle_to_sumo_simulation(self, obstacle: DynamicObstacle) -> bool:
        if _obstacle_is_vehicle(obstacle.obstacle_type):
            return self.vehicles.sync_to_sumo_simulation(
                obstacle,
            )
        elif obstacle.obstacle_type is ObstacleType.PEDESTRIAN:
            return self.pedestrians.sync_to_sumo_simulation(obstacle)
        else:
            raise SumoInterfaceError(
                f"Failed to synchronize dynamic obstacle '{obstacle.obstacle_id}' because its obstacle type '{obstacle.obstacle_type}' is currently not supported"
            )

    def sync_obstacle_from_sumo_simulation(self, obstacle: DynamicObstacle) -> bool:
        if _obstacle_is_vehicle(obstacle.obstacle_type):
            return self.vehicles.sync_from_sumo_simulation(obstacle)
        elif _obstacle_is_pedestrian(obstacle.obstacle_type):
            return self.pedestrians.sync_from_sumo_simulation(obstacle)
        else:
            raise SumoInterfaceError(
                f"Failed to synchronize dynamic obstacle '{obstacle.obstacle_id}' because its obstacle type '{obstacle.obstacle_type}' is currently not supported"
            )

    def sync_to_sumo_simulation(
        self, resource: Union[DynamicObstacle, TrafficLight]
    ) -> bool:
        """
        Sync the state of a dynamic obstacle or a traffic light to SUMO
        """
        if isinstance(resource, DynamicObstacle):
            return self.sync_obstacle_to_sumo_simulation(resource)
        elif isinstance(resource, TrafficLight):
            return self.sync_traffic_light_to_sumo_simulation(resource)
        else:
            raise SumoInterfaceError(
                f"Cannot sync resource of type '{type(resource)}' to SUMO simulation"
            )

    def sync_from_sumo_simulation(
        self, resource: Union[DynamicObstacle, TrafficLight]
    ) -> bool:
        """
        Sync the state of a dynamic obstacle or a traffic light from SUMO
        """
        if isinstance(resource, DynamicObstacle):
            return self.sync_obstacle_from_sumo_simulation(resource)
        elif isinstance(resource, TrafficLight):
            return self.sync_traffic_light_from_sumo_simulation(resource)
        else:
            raise SumoInterfaceError(
                f"Cannot sync resource of type '{type(resource)}' from SUMO simulation"
            )

    def fetch_new_from_sumo_simulation(
        self,
    ) -> List[DynamicObstacle]:
        """
        Fetch new vehicles and pedestrians from the SUMO simulation
        """
        new_vehicles = self.vehicles.fetch_new_from_sumo_simulation()
        new_pedestrians = self.pedestrians.fetch_new_from_sumo_simulation()

        return new_vehicles + new_pedestrians

    def get_collision_events(self) -> List[CollisionEvent]:
        collisions = self._simulation_backend.get_collisions()
        collision_events = []
        for collision in collisions:
            collider_cr_id = self._id_mapper.sumo2cr(collision.collider)
            if collider_cr_id is None:
                # Skip resources we do not track
                _LOGGER.warning(
                    f"SUMO {collider_cr_id} is colliding in simulation, but it is currently not tracked"
                )
                continue

            victim_cr_id = self._id_mapper.sumo2cr(collision.victim)
            if victim_cr_id is None:
                # Skip resources we do not track
                _LOGGER.warning(
                    f"SUMO {victim_cr_id} is colliding in simulation, but it is currently not tracked"
                )
                continue

            collision_type = CollisionEvent.CollisionType.from_sumo_type(collision.type)
            collision_event = CollisionEvent(
                collider_cr_id, victim_cr_id, collision_type
            )
            collision_events.append(collision_event)

        return collision_events

    def get_simulation_events(self) -> Sequence[SimulationEvent]:
        """
        Get events that happened during the current time step.
        """
        collision_events = self.get_collision_events()
        simulation_events: Sequence[SimulationEvent] = collision_events
        return simulation_events
