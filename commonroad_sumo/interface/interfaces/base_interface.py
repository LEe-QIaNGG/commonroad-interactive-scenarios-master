import abc
from typing import Generic, List, TypeVar

from commonroad_sumo.backend import SumoSimulationBackend
from commonroad_sumo.interface.id_mapper import IdMapper

_T = TypeVar("_T")


class BaseInterface(abc.ABC, Generic[_T]):
    """
    An Interface between CommonRoad and SUMO that provides Bi-Directional sync capabilities
    for an arbitrary CommonRoad object of type _T.
    """

    def __init__(self, simulation_backend: SumoSimulationBackend, id_mapper: IdMapper):
        self._simulation_backend = simulation_backend
        self._id_mapper = id_mapper
        self._current_time_step = 0

    def simulate_step(self) -> bool:
        """
        Perform a simulation step. If an interface performs lazy syncs, this method will perform the aggregated syncs.

        :return: Whether this interface performed actual changes in SUMO. Only relevant for lazy interfaces.
        """
        self._current_time_step += 1
        return False

    @abc.abstractmethod
    def fetch_new_from_sumo_simulation(self) -> List[_T]:
        """
        Fetch CommonRoad objects that have entered the SUMO simulation since the last simulation step and are not yet tracked.

        :return: list of new CommonRoad objects
        """
        ...

    @abc.abstractmethod
    def sync_from_sumo_simulation(self, commonroad_object: _T) -> bool:
        """
        Sync the state of the CommonRoad object from the SUMO simulation and directly update the CommonRoad object.

        :param commonroad_object: The CommonRoad object which will be updated.
        :return: Whether new data was synced from the SUMO simulation. If no data was synced this object is no longer part of the simulation.
        """
        ...

    @abc.abstractmethod
    def sync_to_sumo_simulation(self, commonroad_object: _T) -> bool:
        """
        Sync the given CommonRoad object to SUMO according to it's state at the time step of the interface.

        :param commonroad_object: The CommonRoad object that should be synced to the SUMO simulation.
        :return: Whether this interface performed actual changes in SUMO.
        """
