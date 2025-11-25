import abc
import math
from typing import Optional

import numpy as np
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.state import State

from sumocr.interface.id_mapper import IdDomain, IdMapper, SumoId
from sumocr.interface.interfaces.base_interface import BaseInterface
from sumocr.interface.sumo_simulation_backend import SumoSimulationBackend


class DynamicObstacleInterface(BaseInterface[DynamicObstacle], abc.ABC):
    def __init__(
        self,
        simulation_backend: SumoSimulationBackend,
        id_mapper: IdMapper,
        id_domain: IdDomain,
    ):
        super().__init__(simulation_backend, id_mapper)
        # This interface must be paramterized with the IdDomain.
        # This parameterization is not a Generic, as the IdDomain is not a type per se, but a value
        self._id_domain = id_domain

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
        self, sumo_id: SumoId, state: State
    ):
        sumo_angle = 90.0 - math.degrees(state.orientation)
        position = state.position + 0.5 * np.array(
            [math.cos(state.orientation), math.sin(state.orientation)]
        )
        # Forward the position and orientation information to the concret implemention of the move functionality
        self._move_dynamic_obstacle_in_sumo_simulation(
            sumo_id, x=position[0], y=position[1], angle=sumo_angle
        )

    @abc.abstractmethod
    def _move_dynamic_obstacle_in_sumo_simulation(
        self, sumo_id: SumoId, x: float, y: float, angle: float
    ):
        ...

    def _update_dynamic_obstacle_in_sumo_simluation(
        self, sumo_id: SumoId, dynamic_obstacle: DynamicObstacle, time_step: int
    ) -> bool:
        state = dynamic_obstacle.state_at_time(time_step)
        if state is None:
            return False
        self._sync_dynamic_obstacle_state_to_sumo_simulation(sumo_id, state)
        return True

    def sync_to_sumo_simulation(
        self, dynamic_obstacle: DynamicObstacle, time_step: int
    ) -> bool:
        """
        :return: Whether the sync was performed successfully
        """
        sumo_id: Optional[SumoId] = None  # Make mypy happy
        if (
            time_step < dynamic_obstacle.prediction.initial_time_step
            or time_step > dynamic_obstacle.prediction.final_time_step + 1
        ):
            # The dynamic_obstacle has no state definied for the current time_step, therefore it is skipped
            return False
        elif dynamic_obstacle.prediction.initial_time_step == time_step:
            # The first time this dynamic_obstacle appears, so we need to add it to the simulation
            sumo_id = self._id_mapper.new_sumo_id_from_cr_id(
                dynamic_obstacle.obstacle_id, self._id_domain
            )
            self._add_new_dynamic_obstacle_in_sumo_simulation(sumo_id, dynamic_obstacle)
            self._update_dynamic_obstacle_in_sumo_simluation(
                sumo_id, dynamic_obstacle, time_step
            )
        elif dynamic_obstacle.prediction.final_time_step == time_step - 1:
            # The last_time step was the last one for this obstacle, therefore it shall no longer be simulated and must be removed from the simulation
            sumo_id = self._id_mapper.cr2sumo(
                dynamic_obstacle.obstacle_id, self._id_domain
            )

            # sumo_id can be None, but if this is the case this indicates a logic error
            if sumo_id is None:
                raise RuntimeError(
                    f"Tried to remove the dynamic obstacle '{dynamic_obstacle.obstacle_id}' in SUMO, but we have not registered this obstacle yet. This is a bug."
                )

            # Perform the concrete removal, which is implemented by our SubClass
            self._remove_dynamic_obstacle_in_sumo_simulation(sumo_id, dynamic_obstacle)
        else:
            sumo_id = self._id_mapper.cr2sumo(
                dynamic_obstacle.obstacle_id, self._id_domain
            )

            # sumo_id can be None, but if this is the case this indicates a logic error
            if sumo_id is None:
                raise RuntimeError(
                    f"Tried to update the dynamic obstacle '{dynamic_obstacle.obstacle_id}' in SUMO, but we have not registered this obstacle yet. This is a bug."
                )
            self._update_dynamic_obstacle_in_sumo_simluation(
                sumo_id, dynamic_obstacle, time_step
            )

        # Inform the caller, that we performed some form of sync
        return True
