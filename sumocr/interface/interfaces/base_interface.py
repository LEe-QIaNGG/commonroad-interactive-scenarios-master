import abc
from typing import Generic, List, TypeVar

from sumocr.interface.id_mapper import IdMapper, SumoId
from sumocr.interface.sumo_simulation_backend import SumoSimulationBackend

_T = TypeVar("_T")


class BaseInterface(abc.ABC, Generic[_T]):
    def __init__(self, simulation_backend: SumoSimulationBackend, id_mapper: IdMapper):
        self._simulation_backend = simulation_backend
        self._id_mapper = id_mapper

    def simulate_step(self) -> bool:
        return False

    @abc.abstractmethod
    def fetch_new_from_sumo_simulation(self) -> List[SumoId]:
        ...

    # @abc.abstractmethod
    # def sync_from_sumo_simulation(self) -> List[_T]:
    #     ...

    @abc.abstractmethod
    def sync_to_sumo_simulation(self, commonroad_object: _T, time_step: int) -> bool:
        ...
