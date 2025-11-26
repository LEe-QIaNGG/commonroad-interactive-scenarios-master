__all__ = [
    "CommonRoadId",
    "SumoId",
    "IdMapper",
    "get_maximum_commonroad_id",
]

import logging
from typing import Dict, Optional, Set

import numpy as np
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario

from commonroad_sumo.errors import SumoInterfaceError

_LOGGER = logging.getLogger(__name__)

CommonRoadId = int
SumoId = str


def get_maximum_commonroad_id(
    scenario: Scenario, planning_problem_set: Optional[PlanningProblemSet] = None
) -> CommonRoadId:
    """
    Get the largest ID from all objects inside the CommonRoad :param:`scenario` and the optional :param:`planning_problem_set`.
    """
    lanelet_network = scenario.lanelet_network
    max_lanelet = (
        np.max([lanelet.lanelet_id for lanelet in lanelet_network.lanelets])
        if lanelet_network.lanelets
        else 0
    )
    max_intersection = (
        np.max([i.intersection_id for i in lanelet_network.intersections])
        if lanelet_network.intersections
        else 0
    )
    max_traffic_light = (
        np.max([t.traffic_light_id for t in lanelet_network.traffic_lights])
        if lanelet_network.traffic_lights
        else 0
    )
    max_traffic_sign = (
        np.max([t.traffic_sign_id for t in lanelet_network.traffic_signs])
        if lanelet_network.traffic_signs
        else 0
    )
    max_obstacle = (
        np.max([o.obstacle_id for o in scenario.obstacles]) if scenario.obstacles else 0
    )
    max_pp = 0
    if planning_problem_set is not None:
        max_pp = max(list(planning_problem_set.planning_problem_dict.keys()))

    val = np.max(
        [
            max_lanelet,
            max_intersection,
            max_traffic_light,
            max_traffic_sign,
            max_obstacle,
            max_pp,
        ]
    )
    if isinstance(val, np.generic):
        return val.item()
    else:
        return val


class IdMapper:
    """
    IdMapper bi-directionally maps Ids between CommonRoad and SUMO.

    :param id_allocator: Optionally provide an existing IdAllocator that will be used to allocate new CommonRoad IDs. If None is provided, the default `IncrementIdAllocator` will be used.
    """

    def __init__(
        self,
        id_set: Optional[Set[CommonRoadId]] = None,
        max_cr_id: int = 0,
        strict: bool = False,
    ):
        if id_set is None:
            self._id_set = set()
        else:
            self._id_set = id_set
        self._id_ctr = max_cr_id
        self._strict = strict

        self._sumo2cr: Dict[SumoId, CommonRoadId] = {}
        self._cr2sumo: Dict[CommonRoadId, SumoId] = {}

    @classmethod
    def from_scenario(cls, scenario: Scenario, strict: bool = False) -> "IdMapper":
        """
        Create a new IdMapper that generates new CommonRoad Ids that are not yet present in the scenario.
        """
        max_cr_id = get_maximum_commonroad_id(scenario)
        # Make sure to copy the ID set, otherwise we will interfere with the scenario.
        # E.g. we might add the ID of a new dynamic obstacle to the set,
        # but then the corresponding dynamic obstacle is added to the scenario.
        # If the sets are the same, the insertion will fail because the ID is already in the used id set of the scenario.
        id_set = set(scenario._id_set)
        return IdMapper(id_set, max_cr_id, strict)

    def _allocate_new_cr_id(self, sumo_id: SumoId) -> CommonRoadId:
        if sumo_id.isdigit():
            possible_cr_id = int(sumo_id)
            if possible_cr_id not in self._id_set:
                self._id_set.add(possible_cr_id)
                return possible_cr_id
            elif self._strict:
                raise SumoInterfaceError(
                    f"Failed to map the SUMO ID {sumo_id} directly to it a CommonRoad ID because the ID is already used in CommonRoad!"
                )

        while self._id_ctr in self._id_set:
            self._id_ctr += 1

        self._id_set.add(self._id_ctr)

        return self._id_ctr

    def _allocate_new_sumo_id(self, cr_id: CommonRoadId) -> SumoId:
        return str(cr_id)

    def insert_mapping(self, sumo_id: SumoId, cr_id: CommonRoadId) -> None:
        """
        Map the provided sumo_id and cr_id together for the given domain.
        No checks are performed, to verify the validity of the mapping.
        """
        self._sumo2cr[sumo_id] = cr_id
        self._cr2sumo[cr_id] = sumo_id

    def new_cr_id_from_sumo_id(self, sumo_id: SumoId) -> CommonRoadId:
        """
        Create a new CommonRoadId for the sumo_id in the given domain and add it to the internal mapping.
        If the given sumo_id, already has a CommonRoadId allocated, a RuntimeError will be thrown
        """
        if self.has_sumo2cr(sumo_id):
            cr_id = self._sumo2cr[sumo_id]
            raise SumoInterfaceError(
                f"Tried to generate a new CommonRoad ID for the SUMO '{sumo_id}', but the CommonRoad ID already '{cr_id}' is already associated with this SUMO ID"
            )

        cr_id = self._allocate_new_cr_id(sumo_id)

        self.insert_mapping(sumo_id, cr_id)

        return cr_id

    def new_sumo_id_from_cr_id(self, cr_id: CommonRoadId) -> SumoId:
        if self.has_cr2sumo(cr_id):
            sumo_id = self._cr2sumo[cr_id]
            raise SumoInterfaceError(
                f"Tried to generate a new SUMO ID for the CommonRoad ID '{cr_id}', but the SUMO ID '{sumo_id}' is already associated with this CommonRoad ID"
            )

        sumo_id = self._allocate_new_sumo_id(cr_id)

        self.insert_mapping(sumo_id, cr_id)

        return sumo_id

    def has_sumo2cr(self, sumo_id: SumoId) -> bool:
        """
        Check whether the given sumo_id is already mapped to a CommonRoadId in the given domain.
        """
        return sumo_id in self._sumo2cr

    def sumo2cr(self, sumo_id: SumoId) -> Optional[CommonRoadId]:
        """
        Retrive the CommonRoadId that is associated with the given sumo_id from the given domain.

        :return: The associated CommonRoadId, or None if no CommonRoadId could be found
        """
        return self._sumo2cr.get(sumo_id)

    def has_cr2sumo(self, cr_id: CommonRoadId) -> bool:
        """
        Check whether the given cr_id has a sumo_id associated in the given domain
        """
        return cr_id in self._cr2sumo

    def cr2sumo(self, cr_id: CommonRoadId) -> Optional[SumoId]:
        """
        Retrive the SumoId that is associated with the given cr_id from the given domain.

        :return: The associated SumoId, or None if no SumoId could be found
        """
        return self._cr2sumo.get(cr_id)
