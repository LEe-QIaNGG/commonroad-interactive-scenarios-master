import math

from commonroad.scenario.obstacle import ObstacleType

from commonroad_sumo.backend.types import SumoVehicleClass

_VEHICLE_CLASS_CR2SUMO = {
    ObstacleType.UNKNOWN: SumoVehicleClass.PASSENGER,
    ObstacleType.CAR: SumoVehicleClass.PASSENGER,
    ObstacleType.TRUCK: SumoVehicleClass.TRUCK,
    ObstacleType.BUS: SumoVehicleClass.BUS,
    ObstacleType.BICYCLE: SumoVehicleClass.BICYCLE,
    ObstacleType.PEDESTRIAN: SumoVehicleClass.PEDESTRIAN,
    ObstacleType.PRIORITY_VEHICLE: SumoVehicleClass.EMERGENCY,
    ObstacleType.PARKED_VEHICLE: SumoVehicleClass.PASSENGER,
    ObstacleType.TRAIN: SumoVehicleClass.RAIL,
    ObstacleType.MOTORCYCLE: SumoVehicleClass.MOTORCYCLE,
    ObstacleType.TAXI: SumoVehicleClass.TAXI,
    ObstacleType.ROAD_BOUNDARY: SumoVehicleClass.CUSTOM1,
    ObstacleType.BUILDING: SumoVehicleClass.CUSTOM2,
    ObstacleType.PILLAR: SumoVehicleClass.CUSTOM2,
    ObstacleType.MEDIAN_STRIP: SumoVehicleClass.CUSTOM1,
    ObstacleType.CONSTRUCTION_ZONE: SumoVehicleClass.PASSENGER,
}


def cr_obstacle_type_to_sumo_vehicle_class(
    obstacle_type: ObstacleType,
) -> SumoVehicleClass:
    if obstacle_type not in _VEHICLE_CLASS_CR2SUMO:
        valid_obstacle_types = list(_VEHICLE_CLASS_CR2SUMO.keys())
        raise ValueError(
            f"obstacle type '{obstacle_type}' cannot be mapped to a SUMO vehicle class. Valid obstacle types are: {valid_obstacle_types}."
        )

    return _VEHICLE_CLASS_CR2SUMO[obstacle_type]


# CommonRoad obstacle type to sumo vehicle class
_VEHICLE_CLASS_SUMO2CR = {
    SumoVehicleClass.PASSENGER: ObstacleType.CAR,
    SumoVehicleClass.TRUCK: ObstacleType.TRUCK,
    SumoVehicleClass.BUS: ObstacleType.BUS,
    SumoVehicleClass.BICYCLE: ObstacleType.BICYCLE,
    SumoVehicleClass.PEDESTRIAN: ObstacleType.PEDESTRIAN,
    SumoVehicleClass.EMERGENCY: ObstacleType.PRIORITY_VEHICLE,
    SumoVehicleClass.RAIL: ObstacleType.TRAIN,
    SumoVehicleClass.MOTORCYCLE: ObstacleType.MOTORCYCLE,
    SumoVehicleClass.TAXI: ObstacleType.TAXI,
    SumoVehicleClass.CUSTOM2: ObstacleType.PILLAR,
    SumoVehicleClass.CUSTOM1: ObstacleType.MEDIAN_STRIP,
}


def sumo_vehicle_class_to_cr_obstacle_type(
    vehicle_class: SumoVehicleClass,
) -> ObstacleType:
    if vehicle_class not in _VEHICLE_CLASS_SUMO2CR:
        valid_vehicle_classes = list(_VEHICLE_CLASS_SUMO2CR.keys())
        raise ValueError(
            f"vehicle class {vehicle_class} cannot be mapped to a CommonRoad obstacle type. Valid vehicle classes are: {valid_vehicle_classes}"
        )

    return _VEHICLE_CLASS_SUMO2CR[vehicle_class]


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


def calculate_velocity_from_speed(speed: float, lat_speed: float) -> float:
    return math.sqrt(speed**2 + lat_speed**2)


def calculate_orientation_from_angle(angle: float) -> float:
    return math.radians(-angle + 90.0)


def calculate_angle_from_orientation(orientation: float) -> float:
    return 90.0 - math.degrees(orientation)
