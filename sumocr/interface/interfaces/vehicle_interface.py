from typing import List

from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import DynamicObstacle

from sumocr.interface.id_mapper import CommonRoadId, IdDomain, IdMapper, SumoId
from sumocr.interface.interfaces.dynamic_obstacle_interface import (
    DynamicObstacleInterface,
)
from sumocr.interface.sumo_simulation_backend import SumoSimulationBackend
from sumocr.interface.util import cr_obstacle_type_to_sumo_vehicle_class


class VehicleInterface(DynamicObstacleInterface):
    def __init__(self, simulation_backend: SumoSimulationBackend, id_mapper: IdMapper):
        super().__init__(simulation_backend, id_mapper, IdDomain.VEHICLE)

    def _add_new_vehicle_from_sumo_simulation(self, veh_id: SumoId) -> CommonRoadId:
        """
        Adds a new vehicle from SUMO simulation to our internal mapping and returns the create CommonRoadId
        """
        # If this is an ego vehicle, we want to add it to the ego vehicle domain
        # otherwise we consider it as an obstacle vehicle
        primary_domain = (
            IdDomain.EGO_VEHICLE
            if IdDomain.EGO_VEHICLE.contains_sumo_id(veh_id)
            else IdDomain.OBSTACLE_VEHICLE
        )
        cr_id = self._id_mapper.new_cr_id_from_sumo_id(veh_id, primary_domain)
        # Also make sure to add the new vehicle to the "general" vehicle IdDomain
        self._id_mapper.insert_mapping(veh_id, cr_id, IdDomain.VEHICLE)
        return cr_id

    def fetch_new_from_sumo_simulation(self) -> List[SumoId]:
        """
        Retrive all new vehicles that have entered the SUMO simulation, we currenlty do not track and register them in our internal mapping

        :return A list containing the SumoIds of the new vehicles
        """
        new_vehicle_ids = []
        vehicle_ids = self._simulation_backend.vehicle_domain.getIDList()
        for veh_id in vehicle_ids:
            if not self._id_mapper.has_sumo2cr(veh_id, IdDomain.VEHICLE):
                self._add_new_vehicle_from_sumo_simulation(veh_id)
                new_vehicle_ids.append(veh_id)

        return new_vehicle_ids

    def _move_dynamic_obstacle_in_sumo_simulation(
        self, sumo_id: SumoId, x: float, y: float, angle: float
    ):
        self._simulation_backend.move_vehicle(sumo_id, x=x, y=y, angle=angle)

    def _add_new_dynamic_obstacle_in_sumo_simulation(
        self, sumo_id: SumoId, vehicle: DynamicObstacle
    ):
        vehicle_class = cr_obstacle_type_to_sumo_vehicle_class(vehicle.obstacle_type)
        self._simulation_backend.add_vehicle(
            vehicle_id=sumo_id, vehicle_class=vehicle_class
        )
        shape = vehicle.obstacle_shape
        if isinstance(shape, Rectangle):
            self._simulation_backend.vehicle_domain.setLength(sumo_id, shape.length)
            self._simulation_backend.vehicle_domain.setWidth(sumo_id, shape.width)
        else:
            raise ValueError(
                f"Vehicle has shape '{shape}' which we currently cannot handle."
            )

    def _remove_dynamic_obstacle_in_sumo_simulation(
        self, sumo_id: SumoId, _: DynamicObstacle
    ):
        self._simulation_backend.remove_vehicle(sumo_id)
