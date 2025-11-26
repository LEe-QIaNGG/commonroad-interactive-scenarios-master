import logging
import math
from typing import List, Optional

import numpy as np
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType, Shape
from commonroad.scenario.state import ExtendedPMState, InitialState, SignalState
from commonroad.scenario.trajectory import Trajectory

from commonroad_sumo.backend import SumoSimulationBackend
from commonroad_sumo.errors import SumoInterfaceError
from commonroad_sumo.interface.driving_model_parameters_provider import (
    DrivingModelParametersProvider,
)
from commonroad_sumo.interface.id_mapper import IdMapper, SumoId
from commonroad_sumo.interface.interfaces.dynamic_obstacle_interface import (
    DynamicObstacleInterface,
    StateParams,
)
from commonroad_sumo.interface.util import (
    calculate_angle_from_orientation,
    calculate_orientation_from_angle,
    calculate_velocity_from_speed,
    cr_obstacle_type_to_sumo_vehicle_class,
    sumo_vehicle_class_to_cr_obstacle_type,
)

logger = logging.getLogger()


class VehicleInterface(DynamicObstacleInterface):
    def __init__(
        self,
        simulation_backend: SumoSimulationBackend,
        id_mapper: IdMapper,
        driving_model_parameters_provider: Optional[
            DrivingModelParametersProvider
        ] = None,
    ):
        self._driving_model_parameters_provider = driving_model_parameters_provider
        super().__init__(simulation_backend, id_mapper)

    def _fetch_vehicle_signal_state_from_sumo_simulation(
        self, veh_id: SumoId
    ) -> SignalState:
        sumo_signals = self._simulation_backend.get_vehicle_signals(veh_id)
        return SignalState(
            time_step=self._current_time_step,
            indicator_left=sumo_signals.blinker_left,
            indicator_right=sumo_signals.blinker_right,
            braking_lights=sumo_signals.brake_light,
            flashing_blue_lights=sumo_signals.emergency_blue,
        )

    def _fetch_dynamic_obstacle_state_params_from_sumo_simulation(
        self, veh_id: SumoId
    ) -> StateParams:
        """
        Fetch the state of a vehicle from the SUMO simulation
        """
        position = np.array(self._simulation_backend.get_vehicle_position(veh_id))
        speed = self._simulation_backend.get_vehicle_speed(veh_id)
        lat_speed = self._simulation_backend.get_vehicle_lateral_speed(veh_id)
        angle = self._simulation_backend.get_vehicle_angle(veh_id)
        orientation = calculate_orientation_from_angle(angle)
        acceleration = self._simulation_backend.get_vehicle_acceleration(veh_id)
        length, _ = self._simulation_backend.get_vehicle_shape(veh_id)

        position -= (
            0.5 * length * np.array([math.cos(orientation), math.sin(orientation)])
        )

        return {
            "position": position,
            "velocity": calculate_velocity_from_speed(speed, lat_speed),
            "orientation": orientation,
            "acceleration": acceleration,
            "time_step": self._current_time_step,
        }

    def _fetch_vehicle_shape(
        self, veh_id: SumoId, obstacle_type: ObstacleType
    ) -> Shape:
        if self._driving_model_parameters_provider is None:
            length, width = self._simulation_backend.get_vehicle_shape(veh_id)
            return Rectangle(length, width)

        shape = self._driving_model_parameters_provider.get_shape(obstacle_type)
        self._simulation_backend.set_vehicle_shape(veh_id, shape.length, shape.width)
        return shape

    def _calibrate_sumo_vehicle_parameters(
        self, veh_id: SumoId, obstacle_type: ObstacleType
    ):
        if self._driving_model_parameters_provider is None:
            return

        accel = self._driving_model_parameters_provider.get_accel(obstacle_type)
        self._simulation_backend.set_vehicle_acceleration(veh_id, accel)
        decel = self._driving_model_parameters_provider.get_decel(obstacle_type)
        self._simulation_backend.set_vehicle_deceleration(veh_id, decel)
        min_gap = self._driving_model_parameters_provider.get_min_gap(obstacle_type)
        self._simulation_backend.set_vehicle_min_gap(veh_id, min_gap)
        max_speed = self._driving_model_parameters_provider.get_max_speed(obstacle_type)
        self._simulation_backend.set_vehicle_max_speed(veh_id, max_speed)

    def _fetch_new_vehicle_from_sumo_simulation(
        self, veh_id: SumoId
    ) -> DynamicObstacle:
        """
        Fetch a specific vehicle from the SUMO simulation and return it as a CommonRoad DynamicObstacle

        :param: veh_id: The ID of the vehicle in SUMO
        :return The newly created DynamicObstacle
        """
        cr_id = self._id_mapper.new_cr_id_from_sumo_id(veh_id)

        vehicle_class = self._simulation_backend.get_vehicle_class(veh_id)
        obstacle_type = sumo_vehicle_class_to_cr_obstacle_type(vehicle_class)

        obstacle_shape = self._fetch_vehicle_shape(veh_id, obstacle_type)

        self._calibrate_sumo_vehicle_parameters(veh_id, obstacle_type)

        state_params = self._fetch_dynamic_obstacle_state_params_from_sumo_simulation(
            veh_id
        )
        state = InitialState(**state_params)

        signal_state = self._fetch_vehicle_signal_state_from_sumo_simulation(veh_id)
        return DynamicObstacle(
            cr_id,
            obstacle_type=obstacle_type,
            obstacle_shape=obstacle_shape,
            initial_state=state,
            initial_signal_state=signal_state,
            signal_series=[],
            prediction=None,
            history=[],
        )

    def fetch_new_from_sumo_simulation(self) -> List[DynamicObstacle]:
        """
        Retrive all new vehicles, which we currenlty do not track and register them in our internal mapping

        :return A list containing the SumoIds of the new vehicles
        """
        new_vehicles = []
        vehicle_ids = self._simulation_backend.get_vehicle_ids()
        for veh_id in vehicle_ids:
            if not self._id_mapper.has_sumo2cr(veh_id):
                vehicle = self._fetch_new_vehicle_from_sumo_simulation(veh_id)
                new_vehicles.append(vehicle)

        return new_vehicles

    def sync_from_sumo_simulation(self, dynamic_obstacle: DynamicObstacle) -> bool:
        sumo_id = self._id_mapper.cr2sumo(dynamic_obstacle.obstacle_id)
        if sumo_id is None:
            raise SumoInterfaceError(
                f"Tried to sync obstacle '{dynamic_obstacle.obstacle_id}' from SUMO simulation, but the obstacle is not part of the simulation"
            )

        if sumo_id not in self._simulation_backend.get_vehicle_ids():
            # The vehicle is no longer part of the SUMO simulation
            return False

        state_params = self._fetch_dynamic_obstacle_state_params_from_sumo_simulation(
            sumo_id
        )

        # Although we could also use CustomState here, the ExtendedPMState has a much lower (ca. 4x) runtime overhead
        state = ExtendedPMState(**state_params)
        if dynamic_obstacle.prediction is None:
            trajectory = Trajectory(self._current_time_step, state_list=[state])
            dynamic_obstacle.update_prediction(
                TrajectoryPrediction(trajectory, dynamic_obstacle.obstacle_shape)
            )
        else:
            assert isinstance(dynamic_obstacle.prediction, TrajectoryPrediction)
            dynamic_obstacle.prediction.trajectory.append_state(state)

        signal_state = self._fetch_vehicle_signal_state_from_sumo_simulation(sumo_id)
        if dynamic_obstacle.signal_series is None:
            dynamic_obstacle.signal_series = []
        dynamic_obstacle.signal_series.append(signal_state)

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
        self._simulation_backend.move_vehicle(sumo_id, x=x, y=y, angle=angle)
        if velocity is not None:
            self._simulation_backend.set_vehicle_speed(sumo_id, velocity)

    def _add_new_dynamic_obstacle_in_sumo_simulation(
        self, sumo_id: SumoId, vehicle: DynamicObstacle
    ):
        if sumo_id in self._simulation_backend.get_vehicle_ids():
            return

        vehicle_class = cr_obstacle_type_to_sumo_vehicle_class(vehicle.obstacle_type)
        self._simulation_backend.add_vehicle(
            vehicle_id=sumo_id, vehicle_class=vehicle_class
        )

        # Adjust the vehicle shape in SUMO
        shape = vehicle.obstacle_shape
        if isinstance(shape, Rectangle):
            self._simulation_backend.set_vehicle_shape(
                sumo_id, shape.length, shape.width
            )
        else:
            raise ValueError(
                f"Vehicle has shape '{shape}' which is currently not handled."
            )

    def _remove_dynamic_obstacle_in_sumo_simulation(
        self, sumo_id: SumoId, _: DynamicObstacle
    ):
        self._simulation_backend.remove_vehicle(sumo_id)
