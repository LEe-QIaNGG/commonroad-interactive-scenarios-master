from commonroad.scenario.scenario import Scenario

from sumocr.interface.id_mapper import IdMapper
from sumocr.interface.interfaces import (
    PedestrianInterface,
    TrafficlightInterface,
    VehicleInterface,
)
from sumocr.interface.sumo_simulation_backend import SumoSimulationBackend


class SumoSimulationInterface:
    """
    Provides an interface to the SUMO simulation, by exposing several subinterfaces.
    """

    def __init__(
        self,
        simulation_backend: SumoSimulationBackend,
        id_mapper: IdMapper,
        scenario: Scenario,
    ):
        self._simulation_backend = simulation_backend
        self._id_mapper = id_mapper
        self._scenario = scenario

        self._vehicle_interface = VehicleInterface(simulation_backend, id_mapper)
        self._pedestrian_interface = PedestrianInterface(simulation_backend, id_mapper)
        self._traffic_light_interface = TrafficlightInterface(
            simulation_backend, id_mapper, scenario
        )

    def simulate_step(self):
        self._vehicle_interface.simulate_step()
        self._pedestrian_interface.simulate_step()
        self._traffic_light_interface.simulate_step()

    @property
    def vehicles(self):
        return self._vehicle_interface

    @property
    def pedestrians(self):
        return self._pedestrian_interface

    @property
    def traffic_lights(self):
        return self._traffic_light_interface
