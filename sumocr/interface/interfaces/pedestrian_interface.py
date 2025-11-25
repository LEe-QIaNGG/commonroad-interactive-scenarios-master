from typing import List, Optional

from commonroad.scenario.obstacle import DynamicObstacle

from sumocr.interface.id_mapper import IdDomain, IdMapper, SumoId
from sumocr.interface.interfaces.dynamic_obstacle_interface import (
    DynamicObstacleInterface,
)
from sumocr.interface.sumo_simulation_backend import SumoSimulationBackend


class PedestrianInterface(DynamicObstacleInterface):
    def __init__(
        self,
        simulation_backend: SumoSimulationBackend,
        id_mapper: IdMapper,
    ):
        super().__init__(simulation_backend, id_mapper, IdDomain.PEDESTRIAN)
        self._suitable_pedestrian_edge_id: Optional[str] = None

    def fetch_new_from_sumo_simulation(self) -> List[SumoId]:
        """
        Retrive all new pedestrians that have entered the SUMO simulation, we currenlty do not track and register them in our internal mapping

        :return A list containg the SumoIds of the new pedestrians
        """
        person_ids = self._simulation_backend.person_domain.getIDList()
        new_pedestrian_ids = []
        for pedestrian_id in person_ids:
            if not self._id_mapper.has_sumo2cr(pedestrian_id, IdDomain.PEDESTRIAN):
                # initialize new pedestrian
                self._id_mapper.new_cr_id_from_sumo_id(
                    pedestrian_id, IdDomain.PEDESTRIAN
                )
                new_pedestrian_ids.append(pedestrian_id)

        return new_pedestrian_ids

    def _move_dynamic_obstacle_in_sumo_simulation(
        self, sumo_id: SumoId, x: float, y: float, angle: float
    ):
        self._simulation_backend.move_person(sumo_id, x=x, y=y, angle=angle)

    def _find_suitable_pedestrian_edge(self) -> Optional[str]:
        """
        Search for an edge in SUMO that allows pedestrians.

        :return: The edge ID of the first edge, that allows pedestrians
        """
        lane_ids: List[str] = self._simulation_backend.lane_domain.getIDList()
        for lane_id in lane_ids:
            if "pedestrian" in self._simulation_backend.lane_domain.getAllowed(lane_id):
                # This lane allows pedestrians, therefore we have found our suitable edge!
                edge_id: str = self._simulation_backend.lane_domain.getEdgeID(lane_id)
                return edge_id

        return None

    def _add_new_dynamic_obstacle_in_sumo_simulation(
        self, sumo_id: SumoId, pedestrian: DynamicObstacle
    ):
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
