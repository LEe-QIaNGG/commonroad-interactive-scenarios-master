import abc
import math
from typing import Optional, TypedDict

import numpy as np
from commonroad.geometry.shape import Circle, Rectangle
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.state import TraceState

from commonroad_sumo.backend import SumoSimulationBackend
from commonroad_sumo.interface.id_mapper import IdMapper, SumoId
from commonroad_sumo.interface.interfaces.base_interface import BaseInterface


class StateParams(TypedDict):
    time_step: int
    position: np.ndarray
    orientation: float
    acceleration: float
    velocity: float


class DynamicObstacleInterface(BaseInterface[DynamicObstacle], abc.ABC):
    """
    Base interface for synchronizing DynamicObstacles between CommonRoad and SUMO.
    """

    def __init__(
        self,
        simulation_backend: SumoSimulationBackend,
        id_mapper: IdMapper,
    ):
        super().__init__(simulation_backend, id_mapper)

    @abc.abstractmethod
    def _fetch_dynamic_obstacle_state_params_from_sumo_simulation(
        self, obstacle_id: SumoId
    ) -> StateParams:
        # An extra fetch method which just returns the state parameters as a dict. This is necessary, because the state params can be used to construct an initial state and also a "normal" trace state. Therefore, we cannot have a simple method which only returns a CommonRoad State object, because we need to support both state types.
        ...

    @abc.abstractmethod
    def sync_from_sumo_simulation(self, dynamic_obstacle: DynamicObstacle) -> bool: ...

    # Methods for the sync CommonRoad -> SUMO

    @abc.abstractmethod
    def _add_new_dynamic_obstacle_in_sumo_simulation(
        self, sumo_id: SumoId, dynamic_obstacle: DynamicObstacle
    ):
        """
        Perform the concret addition of the dynamic_obstacle into the SUMO simulation.
        After calling this method, the simulation must contain the given obstacle.
        """
        ...

    @abc.abstractmethod
    def _remove_dynamic_obstacle_in_sumo_simulation(
        self, sumo_id: SumoId, dynamic_obstacle: DynamicObstacle
    ):
        """
        Perform the concrete removal of the dynamic_obstacle from the SUMO simulation.
        After calling this method, the simulation must not contain the given obstacle anymore.
        """
        ...

    def _sync_dynamic_obstacle_state_to_sumo_simulation(
        self, sumo_id: SumoId, state: TraceState, dynamic_obstacle: DynamicObstacle
    ):
        length = 0.0
        if isinstance(dynamic_obstacle.obstacle_shape, Rectangle):
            length = dynamic_obstacle.obstacle_shape.length
        elif isinstance(dynamic_obstacle.obstacle_shape, Circle):
            length = dynamic_obstacle.obstacle_shape.radius * 2
        else:
            raise ValueError(
                f"DynamicObstacle '{dynamic_obstacle.obstacle_id}' has shape '{dynamic_obstacle.obstacle_shape}' which is currently not handled"
            )

        # Thanks to the dynamic nature of state objects, extra runtime checks have to be performed to validate that the state we have received can in fact be synchronized

        for attr in ["position", "orientation"]:
            if not state.has_value(attr):
                raise ValueError(
                    f"State '{state}' of dynamic obstacle '{dynamic_obstacle.obstacle_id}' does not have required attribute '{attr}'"
                )

        position = state.position
        if not isinstance(position, np.ndarray):
            raise ValueError(
                f"Position'{position}' in state '{state}' of dynamic obstacle '{dynamic_obstacle.obstacle_id}' is not a discrete value"
            )

        orientation = state.orientation
        if not isinstance(orientation, float):
            raise ValueError(
                f"Orientation '{orientation}' in state '{state}' of dynamic obstacle '{dynamic_obstacle.obstacle_id}' is not a discrete value"
            )

        # In contrast to position and orientation, velocity must no be set
        velocity = None
        if state.has_value("velocity"):
            # If velocity is set, it also must be a discrete type
            velocity = state.velocity
            if not isinstance(velocity, float):
                raise ValueError(
                    f"Velocity '{velocity}' in state '{state}' of dynamic obstacle '{dynamic_obstacle.obstacle_id}' is not a discrete value"
                )

        position = position + 0.5 * length * np.array(
            [math.cos(orientation), math.sin(orientation)]
        )
        # Forward the position and orientation information to the concret implemention of the move functionality
        self._move_dynamic_obstacle_in_sumo_simulation(
            sumo_id,
            x=position[0],
            y=position[1],
            orientation=orientation,
            velocity=velocity,
        )

    @abc.abstractmethod
    def _move_dynamic_obstacle_in_sumo_simulation(
        self,
        sumo_id: SumoId,
        x: float,
        y: float,
        orientation: float,
        velocity: Optional[float] = None,
    ): ...

    def _update_dynamic_obstacle_in_sumo_simluation(
        self, sumo_id: SumoId, dynamic_obstacle: DynamicObstacle
    ) -> bool:
        state = dynamic_obstacle.state_at_time(self._current_time_step)
        if state is None:
            return False
        self._sync_dynamic_obstacle_state_to_sumo_simulation(
            sumo_id, state, dynamic_obstacle
        )
        return True

    def sync_to_sumo_simulation(self, dynamic_obstacle: DynamicObstacle) -> bool:
        """
        Synchronize the state of the dynamic obstacle to the SUMO simulation.
        If the dynamic obstacle

        :return: Whether the sync was performed successfully
        """
        sumo_id: Optional[SumoId] = self._id_mapper.cr2sumo(
            dynamic_obstacle.obstacle_id
        )

        if dynamic_obstacle.state_at_time(self._current_time_step) is None:
            return False

        if sumo_id is None:
            # The first time this dynamic_obstacle appears, so we need to allocate a new SUMO ID
            # and add the dynamic obstacle to the simulation
            sumo_id = self._id_mapper.new_sumo_id_from_cr_id(
                dynamic_obstacle.obstacle_id
            )
            self._add_new_dynamic_obstacle_in_sumo_simulation(sumo_id, dynamic_obstacle)

        return self._update_dynamic_obstacle_in_sumo_simluation(
            sumo_id, dynamic_obstacle
        )
