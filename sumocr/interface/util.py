import enum
import os
import xml.etree.ElementTree as et
from typing import List

import libsumo as traci
from commonroad.scenario.obstacle import ObstacleType, SignalState

__author__ = "Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["ZIM Projekt ZF4086007BZ8"]
__version__ = "2022.1"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad@lists.lrz.de"
__status__ = "Released"

# CommonRoad obstacle type to sumo type
_VEHICLE_TYPE_CR2SUMO = {
    ObstacleType.UNKNOWN: "passenger",
    ObstacleType.CAR: "passenger",
    ObstacleType.TRUCK: "truck",
    ObstacleType.BUS: "bus",
    ObstacleType.BICYCLE: "bicycle",
    ObstacleType.PEDESTRIAN: "pedestrian",
    ObstacleType.PRIORITY_VEHICLE: "vip",
    ObstacleType.PARKED_VEHICLE: "passenger",
    ObstacleType.CONSTRUCTION_ZONE: "passenger",
    ObstacleType.TRAIN: "rail",
    ObstacleType.ROAD_BOUNDARY: "custom2",
    ObstacleType.MOTORCYCLE: "motorcycle",
    ObstacleType.TAXI: "taxi",
    ObstacleType.BUILDING: "custom2",
    ObstacleType.PILLAR: "custom2",
    ObstacleType.MEDIAN_STRIP: "custom1",
}


def cr_obstacle_type_to_sumo_vehicle_class(obstacle_type: ObstacleType) -> str:
    if obstacle_type not in _VEHICLE_TYPE_CR2SUMO:
        valid_obstacle_types = list(_VEHICLE_TYPE_CR2SUMO.keys())
        raise ValueError(
            f"obstacle type '{obstacle_type}' cannot be mapped to a SUMO vehicle class. Valid obstacle types are: {valid_obstacle_types}."
        )

    return _VEHICLE_TYPE_CR2SUMO[obstacle_type]


# CommonRoad obstacle type to sumo type
_VEHICLE_TYPE_SUMO2CR = {
    "DEFAULT_PEDTYPE": ObstacleType.PEDESTRIAN,
    "passenger": ObstacleType.CAR,
    "truck": ObstacleType.TRUCK,
    "bus": ObstacleType.BUS,
    "bicycle": ObstacleType.BICYCLE,
    "pedestrian": ObstacleType.PEDESTRIAN,
    "vip": ObstacleType.PRIORITY_VEHICLE,
    "rail": ObstacleType.TRAIN,
    "motorcycle": ObstacleType.MOTORCYCLE,
    "taxi": ObstacleType.TAXI,
    "custom2": ObstacleType.PILLAR,
    "custom1": ObstacleType.MEDIAN_STRIP,
}


def sumo_vehicle_class_to_cr_obstacle_type(vehicle_class: str) -> ObstacleType:
    if vehicle_class not in _VEHICLE_TYPE_SUMO2CR:
        valid_vehicle_classes = list(_VEHICLE_TYPE_SUMO2CR.keys())
        raise ValueError(
            f"vehicle class {vehicle_class} cannot be mapped to a CommonRoad obstacle type. Valid vehicle classes are: {valid_vehicle_classes}"
        )

    return _VEHICLE_TYPE_SUMO2CR[vehicle_class]


def get_route_files(config_file) -> List[str]:
    """
    Returns net-file and route-files specified in the config file.

    :param config_file: SUMO config file (.sumocfg)

    """
    if not os.path.isfile(config_file):
        raise FileNotFoundError(config_file)
    tree = et.parse(config_file)
    file_directory = os.path.dirname(config_file)

    # find route-files
    all_route_files = tree.findall("*/route-files")
    route_files = []
    if len(all_route_files) < 1:
        raise RouteError()
    for item in all_route_files:
        attributes = item.attrib["value"].split(",")
        for route in attributes:
            route_files.append(os.path.join(file_directory, route))
    return route_files


traci_subscription_values = (
    traci.constants.VAR_POSITION,
    traci.constants.VAR_SPEED,
    traci.constants.VAR_SPEED_LAT,
    traci.constants.VAR_ACCELERATION,
    traci.constants.VAR_ANGLE,
    traci.constants.VAR_SIGNALS,
)


class SumoSignalIndices(enum.IntEnum):
    """All interpretations with their respective bit indices
    ref.: https://sumo.dlr.de/docs/TraCI/Vehicle_Signalling.html"""

    VEH_SIGNAL_BLINKER_RIGHT = 0
    VEH_SIGNAL_BLINKER_LEFT = 1
    VEH_SIGNAL_BLINKER_EMERGENCY = 2
    VEH_SIGNAL_BRAKELIGHT = 3
    VEH_SIGNAL_FRONTLIGHT = 4
    VEH_SIGNAL_FOGLIGHT = 5
    VEH_SIGNAL_HIGHBEAM = 6
    VEH_SIGNAL_BACKDRIVE = 7
    VEH_SIGNAL_WIPER = 8
    VEH_SIGNAL_DOOR_OPEN_LEFT = 9
    VEH_SIGNAL_DOOR_OPEN_RIGHT = 10
    VEH_SIGNAL_EMERGENCY_BLUE = 11
    VEH_SIGNAL_EMERGENCY_RED = 12
    VEH_SIGNAL_EMERGENCY_YELLOW = 13


max_signal_index: int = max([s.value for s in SumoSignalIndices])


_defined_signals = {
    # only the following signals are computed on every time step
    SumoSignalIndices.VEH_SIGNAL_BLINKER_LEFT: "indicator_left",
    SumoSignalIndices.VEH_SIGNAL_BLINKER_RIGHT: "indicator_right",
    SumoSignalIndices.VEH_SIGNAL_BRAKELIGHT: "braking_lights",
    SumoSignalIndices.VEH_SIGNAL_EMERGENCY_BLUE: "flashing_blue_lights",
}


def get_signal_state(state: int, time_step: int) -> SignalState:
    """
    Computes the CR Signal state from the sumo signals
    """
    binary = list(reversed(bin(state)[2:]))
    bit_string: List[bool] = [
        binary[i] == "1" if i < len(binary) else False
        for i in range(max_signal_index + 1)
    ]

    args = {
        cr_name: bit_string[sumo_name.value]
        for sumo_name, cr_name in _defined_signals.items()
    }
    return SignalState(**{**args, **{"time_step": time_step}})


class NetError(Exception):
    """
    Exception raised if there is no net-file or multiple net-files.

    """

    def __init__(self, len):
        self.len = len

    def __str__(self):
        if self.len == 0:
            return repr("There is no net-file.")
        else:
            return repr("There are more than one net-files.")


class RouteError(Exception):
    """
    Exception raised if there is no route-file.

    """

    def __str__(self):
        return repr("There is no route-file.")


class EgoCollisionError(Exception):
    """
    Exception raised if the ego vehicle collides with another vehicle

    """

    def __init__(self, time_step=None):
        super().__init__()
        self.time_step = time_step

    def __str__(self):
        if self.time_step is not None:
            return repr(
                f"Ego vehicle collides at current simulation step = {self.time_step}!"
            )
        else:
            return repr("Ego vehicle collides at current simulation step!")
