import math
from typing import List, Optional

import numpy as np
from commonroad.geometry.shape import Circle
from commonroad.prediction.prediction import Trajectory, TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.state import CustomState, InitialState

from commonroad_sumo.backend import SumoSimulationBackend
from commonroad_sumo.interface.id_mapper import IdMapper, SumoId
from commonroad_sumo.interface.interfaces.dynamic_obstacle_interface import (
    DynamicObstacleInterface,
    StateParams,
)
from commonroad_sumo.interface.util import (
    calculate_angle_from_orientation,
    calculate_orientation_from_angle,
)


class PedestrianInterface(DynamicObstacleInterface):
    def __init__(
        self,
        simulation_backend: SumoSimulationBackend,
        id_mapper: IdMapper,
    ):
        super().__init__(simulation_backend, id_mapper)
        self._suitable_pedestrian_edge_id: Optional[str] = None

    def _fetch_dynamic_obstacle_state_params_from_sumo_simulation(
        self,
        pedestrian_id: SumoId,
    ) -> StateParams:
        position = np.array(self._simulation_backend.get_person_position(pedestrian_id))
        velocity = self._simulation_backend.get_person_speed(pedestrian_id)
        angle = self._simulation_backend.get_person_angle(pedestrian_id)
        orientation = calculate_orientation_from_angle(angle)
        length = self._simulation_backend.get_person_shape(pedestrian_id)
        acceleration = self._simulation_backend.get_person_acceleration(pedestrian_id)

        position -= (
            0.5 * length * np.array([math.cos(orientation), math.sin(orientation)])
        )

        return {
            "position": position,
            "orientation": orientation,
            "velocity": velocity,
            "acceleration": acceleration,
            "time_step": self._current_time_step,
        }

    def _fetch_new_pedestrian_from_sumo_simulation(
        self,
        pedestrian_id: SumoId,
    ) -> DynamicObstacle:
        cr_id = self._id_mapper.new_cr_id_from_sumo_id(pedestrian_id)
        length = self._simulation_backend.get_person_shape(pedestrian_id)
        obstacle_shape = Circle(length / 2)

        state_params = self._fetch_dynamic_obstacle_state_params_from_sumo_simulation(
            pedestrian_id,
        )
        state = InitialState(**state_params)

        return DynamicObstacle(
            cr_id,
            obstacle_type=ObstacleType.PEDESTRIAN,
            obstacle_shape=obstacle_shape,
            initial_state=state,
            prediction=None,
            history=[],
        )

    def fetch_new_from_sumo_simulation(self) -> List[DynamicObstacle]:
        """
        Retrive all new pedestrians that have entered the SUMO simulation, we currenlty do not track and register them in our internal mapping

        :return A list containg the SumoIds of the new pedestrians
        """
        person_ids = self._simulation_backend.get_person_ids()
        new_pedestrians = []
        for pedestrian_id in person_ids:
            if not self._id_mapper.has_sumo2cr(pedestrian_id):
                # initialize new pedestrian
                pedestrian = self._fetch_new_pedestrian_from_sumo_simulation(
                    pedestrian_id
                )
                new_pedestrians.append(pedestrian)

        return new_pedestrians

    def sync_from_sumo_simulation(self, dynamic_obstacle: DynamicObstacle) -> bool:
        sumo_id = self._id_mapper.cr2sumo(dynamic_obstacle.obstacle_id)
        if sumo_id is None:
            raise RuntimeError(
                f"Tried to sync pedestrian '{dynamic_obstacle.obstacle_id}' from SUMO simulation, but the pedestrian is not part of the simulation"
            )

        if sumo_id not in self._simulation_backend.get_person_ids():
            # The vehicle is no longer part of the SUMO simulation
            return False

        state_params = self._fetch_dynamic_obstacle_state_params_from_sumo_simulation(
            sumo_id
        )
        state = CustomState(**state_params)
        if dynamic_obstacle.prediction is None:
            trajectory = Trajectory(self._current_time_step, state_list=[state])
            prediction = TrajectoryPrediction(
                trajectory, dynamic_obstacle.obstacle_shape
            )
            dynamic_obstacle.prediction = prediction
        else:
            assert isinstance(dynamic_obstacle.prediction, TrajectoryPrediction)
            dynamic_obstacle.prediction.trajectory.append_state(state)

        return True

    def _move_dynamic_obstacle_in_sumo_simulation(
        self,
        sumo_id: SumoId,
        x: float,
        y: float,
        orientation: float,
        velocity: Optional[float] = None,
    ):
        angle = calculate_angle_from_orientation(orientation)
        self._simulation_backend.move_person(sumo_id, x=x, y=y, angle=angle)

        if velocity is not None:
            self._simulation_backend.set_person_speed(sumo_id, velocity)

    def _find_suitable_pedestrian_edge(self) -> Optional[str]:
        """
        Search for an edge in SUMO that allows pedestrians.

        :return: The edge ID of the first edge, that allows pedestrians
        """
        lane_ids = self._simulation_backend.get_lane_ids()
        for lane_id in lane_ids:
            allowed = self._simulation_backend.get_lane_allowed(lane_id)
            # According to https://sumo.dlr.de/pydoc/traci._lane.html#LaneDomain-getAllowed an empy list indicates that all are allowed
            if len(allowed) == 0 or "pedestrian" in allowed:
                # This lane allows pedestrians, therefore we have found our suitable edge!
                edge_id: str = self._simulation_backend.get_edge_id_of_lane(lane_id)
                return edge_id

        return None

    def _add_new_dynamic_obstacle_in_sumo_simulation(
        self, sumo_id: SumoId, pedestrian: DynamicObstacle
    ):
        if sumo_id in self._simulation_backend.get_person_ids():
            raise RuntimeError(
                "Tried to add a pedestrian that is already part of the simulation"
            )
        # Make sure that we have an ID of an edge that allows pedestrians
        # so that the simulation backend will be able to add it to the simulation.
        # If we do not provide a valid edge ID, the addition will fail.
        # The concret edge is not relevant, and therefore we just choose the first one we find.
        # The concret position of the pedestrian will be set by subsequent calls to the move method.
        edge_id = self._suitable_pedestrian_edge_id
        if edge_id is None:
            # This must be the first call to add, so we need to find a suitable pedestrian edge
            edge_id = self._find_suitable_pedestrian_edge()

            # TODO: Check if a valid edge ID is returned, even when there are only sidewalks
            if edge_id is None:
                raise RuntimeError(
                    "Failed to add pedestrian to SUMO simluation: No suitable edge found in SUMO simulation that allows pedestrians."
                )

            # Save the edge id for consecutive calls
            self._suitable_pedestrian_edge_id = edge_id

        self._simulation_backend.add_person(sumo_id, edge_id)

    def _remove_dynamic_obstacle_in_sumo_simulation(
        self, sumo_id: SumoId, _: DynamicObstacle
    ):
        self._simulation_backend.remove_person(sumo_id)
