import warnings
from enum import Enum
from typing import Dict, Optional, Union

CommonRoadId = int
SumoId = str


class IdDomain(Enum):
    """
    Different seperate domains under which IDs can be registered, such that,
    duplicate SUMO IDs are possible, as long as they reside in different domains.
    """

    EGO_VEHICLE = "egoVehicle"
    OBSTACLE_VEHICLE = "obstacleVehicle"
    # Normally all ego vehicles and obstacle vehicles, also fall into the VEHICLE domain.
    # This seperate domains exists, to provide a way to perform lookups against all vehicles, regardless of whether they are ego or obstacle
    VEHICLE = "vehicle"
    PEDESTRIAN = "pedestrian"
    TRAFFIC_LIGHT = "trafficLight"

    def cr_prefix(self) -> CommonRoadId:
        if self == IdDomain.EGO_VEHICLE:
            return 4
        elif self == IdDomain.OBSTACLE_VEHICLE:
            return 3
        elif self == IdDomain.PEDESTRIAN:
            return 5
        elif self == IdDomain.TRAFFIC_LIGHT:
            return 6
        else:
            return 9

    def sumo_prefix(self) -> SumoId:
        return self.value

    def contains_sumo_id(self, id_: SumoId):
        """
        Check whether the domain, contains the given SumoId
        """
        return id_.startswith(self.sumo_prefix())

    def construct_sumo_id(self, base_id: Union[int, str]) -> str:
        """
        Construct a SumoId from the base_id, such that it can be associated with the IdDomain (usualy through `contains_sumo_id`)
        """
        return self.sumo_prefix() + str(base_id)


class IdMapper:
    """
    IdMapper bi-directionally maps Ids between CommonRoad and
    """

    def __init__(self):
        self._sumo2cr: Dict[IdDomain, Dict[SumoId, CommonRoadId]] = {}
        self._cr2sumo: Dict[IdDomain, Dict[CommonRoadId, SumoId]] = {}

        # Initialize the seperate domain mappings
        for domain in IdDomain:
            self._sumo2cr[domain] = {}
            self._cr2sumo[domain] = {}

    def insert_mapping(
        self, sumo_id: SumoId, cr_id: CommonRoadId, domain: IdDomain
    ) -> None:
        """
        Map the provided sumo_id and cr_id together for the given domain.
        No checks are performed, to verify the validatity of the mapping.
        """
        self._sumo2cr[domain][sumo_id] = cr_id
        self._cr2sumo[domain][cr_id] = sumo_id

    def _generate_cr_id(self, domain: IdDomain) -> CommonRoadId:
        """
        Generate a new CommonRoadId that is unique in it's domain
        """
        # TODO: more advanced ID allocations, with a global allocation check
        max_cr_id_in_domain = (
            # If there are already entries, than we use the maximum value
            max(list(self._sumo2cr[domain].values()))
            if len(self._sumo2cr[domain]) > 0
            # Base case: if this is the first entry, just use the prefix as the base
            else domain.cr_prefix() * 1000
        )
        return max_cr_id_in_domain + 1

    def new_cr_id_from_sumo_id(self, sumo_id: SumoId, domain: IdDomain) -> CommonRoadId:
        """
        Create a new CommonRoadId for the sumo_id in the given domain and add it to the internal mapping.
        If the given sumo_id, already has a CommonRoadId allocated, this Id will be returned instead, and no new one will be generated.
        """
        if self.has_sumo2cr(sumo_id, domain):
            cr_id = self._sumo2cr[domain][sumo_id]
            warnings.warn(
                f"For the sumo_id '{sumo_id}' there is already the CommonRoad id '{cr_id}'. No new CommonRoad ID will be generated"
            )
            return cr_id

        cr_id = self._generate_cr_id(domain)

        self.insert_mapping(sumo_id, cr_id, domain)

        return cr_id

    def has_sumo2cr(self, sumo_id: SumoId, domain: IdDomain) -> bool:
        """
        Check whether the given sumo_id is already mapped to a CommonRoadId in the given domain.
        """
        return sumo_id in self._sumo2cr[domain]

    def sumo2cr(self, sumo_id: SumoId, domain: IdDomain) -> Optional[CommonRoadId]:
        """
        Retrive the CommonRoadId that is associated with the given sumo_id from the given domain.

        :return: The associated CommonRoadId, or None if no CommonRoadId could be found
        """
        return self._sumo2cr[domain].get(sumo_id)

    def has_cr2sumo(self, cr_id: CommonRoadId, domain: IdDomain) -> bool:
        """
        Check whether the given cr_id has a sumo_id associated in the given domain
        """
        return cr_id in self._cr2sumo[domain]

    def cr2sumo(self, cr_id: CommonRoadId, domain: IdDomain) -> Optional[SumoId]:
        """
        Retrive the SumoId that is associated with the given cr_id from the given domain.

        :return: The associated SumoId, or None if no SumoId could be found
        """
        return self._cr2sumo[domain].get(cr_id)

    def new_sumo_id_from_cr_id(self, cr_id: CommonRoadId, domain: IdDomain) -> SumoId:
        if self.has_cr2sumo(cr_id, domain):
            warnings.warn(
                f"Tried to generate a new SUMO id for CommonRoad ID '{cr_id}' in domain '{domain}', but there exists already the SUMO ID '{self.cr2sumo(cr_id, domain)}'"
            )
            return self._cr2sumo[domain][cr_id]

        sumo_id = domain.construct_sumo_id(cr_id)

        self.insert_mapping(sumo_id, cr_id, domain)

        return sumo_id
