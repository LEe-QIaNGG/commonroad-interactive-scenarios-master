from abc import ABC, abstractmethod
from enum import Enum, auto

from commonroad.scenario.scenario import Scenario

from commonroad_sumo.sumolib.sumo_project import SumoProject


class SumoTrafficGenerationMode(Enum):
    """Enumeration to represent different traffic conversion modes for SUMO simulations."""

    RANDOM = auto()
    """Generates purely random traffic on a SUMO network.

    This mode creates arbitrary vehicle flows without following specific demand
    or trajectory patterns. It's useful if no data is available for a network.
    """

    RANDOM_TRIPS = auto()
    """Generates purely random traffic on a SUMO network with 'randomTrips' utility from SUMO.

    This mode creates arbitrary vehicle flows without following specific demand
    or trajectory patterns. It's useful if no data is available for a network.
    """

    DEMAND = auto()
    """Generates traffic based on demand models from a scenario.

    This mode creates traffic flows based on origin-destination pairs,
    for incoming and outgoing lanelets.
    """

    INFRASTRUCTURE = auto()
    """Generates traffic based on demand models derived from the capacity of the lanelet network."""

    SAFE_RESIMULATION = auto()
    """Generates traffic based on predefined trajectories of dynamic obstacles.

    This mode creates specific, repeatable routes by following exact trajectory
    data. It's usefull resimulation of existing scenarios or for testing motion planners,
    in conjunction with SUMO.
    """

    UNSAFE_RESIMULATION = auto()
    """Generates traffic based on predefined trajectories of dynamic obstacles but disables all insertion checks.

    This mode creates specific, repeatable routes by following exact trajectory
    data. It's usefull resimulation of existing scenarios or for testing motion planners,
    in conjunction with SUMO.
    """


class AbstractTrafficGenerator(ABC):
    def __init__(self) -> None:
        pass

    @abstractmethod
    def generate_traffic(
        self, scenario: Scenario, sumo_project: SumoProject
    ) -> bool: ...
