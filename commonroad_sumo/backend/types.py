from abc import ABC, abstractmethod
from dataclasses import dataclass, field, fields
from enum import Enum, auto
from typing import Any, Mapping, Sequence

import libsumo

# Type alias that should be used instead of a 'str' type when a SUMO identifier is needed
SumoId = str


@dataclass
class Collision:
    """
    Represents a collision in the SUMO simulation between a `collider` and a `victim`.
    """

    collider: SumoId
    collider_type: str
    victim: SumoId
    victim_type: str
    type: str

    # Wrap a SUMO collision object to provide a pythonic abstraction (type hints and correct naming)
    @classmethod
    def from_sumo_collision(
        cls, sumo_collision: libsumo._simulation.Collision
    ) -> "Collision":
        return Collision(
            collider=sumo_collision.collider,
            collider_type=sumo_collision.colliderType,
            victim=sumo_collision.victim,
            victim_type=sumo_collision.victimType,
            type=sumo_collision.type,
        )


@dataclass
class SumoSpeedMode:
    """
    Usable abstraction on top of the speed mode bitmask:
    https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html#speed_mode_0xb3
    """

    safe_speed: bool = True
    """Regard safe speed"""

    maximum_acceleration: bool = True
    """Regard maximum acceleration"""

    maximum_deceleration: bool = True
    """Regard maximum deceleration"""

    right_of_way_approach: bool = True
    """Regard right of way at intersections (only applies to approaching foe vehicles outside the intersection)"""

    red_light: bool = True
    """Brake hard to avoid passing a red light"""

    right_of_way_inside: bool = True
    """Regard right of way within intersections (only applies to foe vehicles that have entered the intersection)."""

    @classmethod
    def from_sumo_bitset(cls, bitset: int) -> "SumoSpeedMode":
        """
        Convert a SUMO speed mode bitmask to SpeedMode
        """
        # bit0
        safe_speed = bool(bitset & 0b00001)
        # bit1
        maximum_acceleration = bool(bitset & 0b00010)
        # bit2
        maximum_deceleration = bool(bitset & 0b00100)
        # bit3
        right_of_way_approach = bool(bitset & 0b001000)
        # bit4
        red_light = bool(bitset & 0b010000)
        # bit5 (is inverted)
        right_of_way_inside = not bool(bitset & 0b100000)

        return cls(
            safe_speed,
            maximum_acceleration,
            maximum_deceleration,
            right_of_way_approach,
            red_light,
            right_of_way_inside,
        )

    def to_sumo_bitset(self) -> int:
        """
        Export this SpeedMode configuration as a SUMO speed mode bitmask
        """
        # bit5 (right of way inside) is inverted
        bitset = int(not self.right_of_way_inside)
        # all other bits are 'normal'
        # bit4
        bitset = (bitset << 1) | int(self.red_light)
        # bit3
        bitset = (bitset << 1) | int(self.right_of_way_approach)
        # bit2
        bitset = (bitset << 1) | int(self.maximum_deceleration)
        # bit1
        bitset = (bitset << 1) | int(self.maximum_acceleration)
        # bit0
        bitset = (bitset << 1) | int(self.safe_speed)
        return bitset


@dataclass
class SumoSignalState:
    """
    Representation of the signal state of a vehicle in SUMO.
    """

    blinker_right: bool = False
    blinker_left: bool = False
    blinker_emergency: bool = False
    brake_light: bool = False
    front_light: bool = False
    foglight: bool = False
    high_beam: bool = False
    backdrive: bool = False
    wiper: bool = False
    door_open_left: bool = False
    door_open_right: bool = False
    emergency_blue: bool = False
    emergency_yellow: bool = False
    emergency_red: bool = False

    @classmethod
    def from_sumo_bitset(cls, bitset: int) -> "SumoSignalState":
        # bit0
        blinker_right = bool(bitset & 0b1)
        bitset >>= 1
        # bit1
        blinker_left = bool(bitset & 0b1)
        bitset >>= 1
        # bit2
        blinker_emergency = bool(bitset & 0b1)
        bitset >>= 1
        # bit3
        brake_light = bool(bitset & 0b1)
        bitset >>= 1
        # bit4
        front_light = bool(bitset & 0b1)
        bitset >>= 1
        # bit5
        foglight = bool(bitset & 0b1)
        bitset >>= 1
        # bit6
        high_beam = bool(bitset & 0b1)
        bitset >>= 1
        # bit7
        backdrive = bool(bitset & 0b1)
        bitset >>= 1
        # bit8
        wiper = bool(bitset & 0b1)
        bitset >>= 1
        # bit9
        door_open_left = bool(bitset & 0b1)
        bitset >>= 1
        # bit10
        door_open_right = bool(bitset & 0b1)
        bitset >>= 1
        # bit11
        emergency_blue = bool(bitset & 0b1)
        bitset >>= 1
        # bit12
        emergency_yellow = bool(bitset & 0b1)
        bitset >>= 1
        # bit13
        emergency_red = bool(bitset & 0b1)

        return cls(
            blinker_right,
            blinker_left,
            blinker_emergency,
            brake_light,
            front_light,
            foglight,
            high_beam,
            backdrive,
            wiper,
            door_open_left,
            door_open_right,
            emergency_blue,
            emergency_yellow,
            emergency_red,
        )

    def to_sumo_bitset(self) -> int:
        bitset = int(self.emergency_red)
        bitset = (bitset << 1) | int(self.emergency_yellow)
        bitset = (bitset << 1) | int(self.emergency_blue)
        bitset = (bitset << 1) | int(self.door_open_right)
        bitset = (bitset << 1) | int(self.door_open_left)
        bitset = (bitset << 1) | int(self.wiper)
        bitset = (bitset << 1) | int(self.backdrive)
        bitset = (bitset << 1) | int(self.high_beam)
        bitset = (bitset << 1) | int(self.foglight)
        bitset = (bitset << 1) | int(self.front_light)
        bitset = (bitset << 1) | int(self.brake_light)
        bitset = (bitset << 1) | int(self.blinker_emergency)
        bitset = (bitset << 1) | int(self.blinker_left)
        bitset = (bitset << 1) | int(self.blinker_right)
        return bitset


class SumoVehicleClass(Enum):
    """
    These classes are used in lane definitions and allow/disallow the usage of lanes for certain vehicle types.

    Default SUMO vehicle classes as definied in:
    https://sumo.dlr.de/docs/Vehicle_Type_Parameter_Defaults.html
    """

    # Pedestrians and Two-Wheelers
    PEDESTRIAN = "pedestrian"
    BICYCLE = "bicycle"
    MOPED = "moped"
    MOTORCYCLE = "motorcycle"
    SCOOTER = "scooter"
    # Passenger Cars and Light Delivery
    PASSENGER = "passenger"
    TAXI = "traxi"
    E_VEHICLE = "evehicle"
    EMERGENCY = "emergency"
    DELIVERY = "delivery"
    # Trucks and Busses
    TRUCK = "truck"
    TRAILER = "trailer"
    BUS = "bus"
    COACH = "coach"
    # Rail
    TRAM = "tram"
    RAIL_URBAN = "rail_urban"
    RAIL = "rail"
    RAIL_ELECTRIC = "rail_electric"
    RAIL_FAST = "rail_fast"
    # Other
    SHIP = "ship"
    SUBWAY = "subway"
    AIRCRAFT = "aircraft"
    CONTAINER = "container"
    DRONE = "drone"
    WHEELCHAIR = "wheelchair"
    CABLE_CAR = "cable_car"
    CUSTOM1 = "custom1"
    CUSTOM2 = "custom2"

    @classmethod
    def from_sumo_str(cls, vehicle_class_str: str) -> "SumoVehicleClass":
        try:
            return cls(vehicle_class_str)
        except ValueError:
            raise ValueError(f"Unkown SUMO vehicle class '{vehicle_class_str}'")


class SumoLaneChangeReasonConfig(Enum):
    """
    Possible configuration values for the different lane change reasons in :class:`SumoLaneChangeMode`
    """

    DISABLE = 0b00
    RESPECT_TRACI = 0b01
    OVERRIDE_TRACI = 0b10

    def to_sumo_bitset(self) -> int:
        if self == SumoLaneChangeReasonConfig.DISABLE:
            return 0b00
        elif self == SumoLaneChangeReasonConfig.RESPECT_TRACI:
            return 0b01
        else:
            return 0b10

    @classmethod
    def from_sumo_bitset(cls, bitset: int) -> "SumoLaneChangeReasonConfig":
        if bitset == 0b01:
            return cls.RESPECT_TRACI
        elif bitset == 0b10:
            return cls.OVERRIDE_TRACI
        elif bitset == 0b00:
            return cls.DISABLE
        else:
            raise ValueError(
                f"Invalid bitset '{bitset}' for {str(cls)}: Valid bitsets are '00', '01' and '10'"
            )


# The 'respect' bitset used in SumoLaneChangeMode is special, because it contains combinations that are hard to represent with individual values. Therefore, a enum is used to provide a more usable abstraction.
class SumoLaneChangeModeRespect(Enum):
    NO_RESPECT = auto()
    """Do not respect other drivers when following TraCI requests, adapt speed to fulfill request"""

    AVOID_COLLISION = auto()
    """Avoid immediate collisions when following a TraCI request, adapt speed to fulfill request"""

    RESPECT_WITH_SPEED_ADAPTION = auto()
    """Respect the speed / brake gaps of others when changing lanes, adapt speed to fulfill reques"""

    RESPECT_WITH_NO_SPEED_ADAPTION = auto()
    """Respect the speed / brake gaps of others when changing lanes, no speed adaption"""

    def to_sumo_bitset(self) -> int:
        if self == SumoLaneChangeModeRespect.NO_RESPECT:
            return 0b00
        elif self == SumoLaneChangeModeRespect.AVOID_COLLISION:
            return 0b01
        elif self == SumoLaneChangeModeRespect.RESPECT_WITH_SPEED_ADAPTION:
            return 0b10
        else:
            return 0b11

    @classmethod
    def from_sumo_bitset(cls, bitset: int) -> "SumoLaneChangeModeRespect":
        if bitset == 0b00:
            return SumoLaneChangeModeRespect.NO_RESPECT
        elif bitset == 0b01:
            return SumoLaneChangeModeRespect.AVOID_COLLISION
        elif bitset == 0b10:
            return SumoLaneChangeModeRespect.RESPECT_WITH_SPEED_ADAPTION
        elif bitset == 0b11:
            return SumoLaneChangeModeRespect.RESPECT_WITH_NO_SPEED_ADAPTION
        else:
            raise ValueError(
                f"Invalid bitset '{bitset}' for {str(cls)}: Valid bitsets are '00', '01', '10', '11'"
            )


@dataclass
class SumoLaneChangeMode:
    """
    Abstract representation of the lane change bitset

    https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html#lane_change_mode_0xb6
    """

    strategic: SumoLaneChangeReasonConfig = SumoLaneChangeReasonConfig.RESPECT_TRACI
    """Change lanes to continue the route, because the current lane is 'dead'"""

    cooperative: SumoLaneChangeReasonConfig = SumoLaneChangeReasonConfig.RESPECT_TRACI
    """Change in order to allow others to change"""

    speed_gain: SumoLaneChangeReasonConfig = SumoLaneChangeReasonConfig.RESPECT_TRACI
    """The other lane allows for faster driving"""

    obligation: SumoLaneChangeReasonConfig = SumoLaneChangeReasonConfig.RESPECT_TRACI
    """Obligation to drive on the right"""

    respect: SumoLaneChangeModeRespect = (
        SumoLaneChangeModeRespect.RESPECT_WITH_SPEED_ADAPTION
    )
    """Control if vehicles respect other vehicles during their lane change"""

    sublane: SumoLaneChangeReasonConfig = SumoLaneChangeReasonConfig.RESPECT_TRACI
    """Control sublane changes"""

    def to_sumo_bitset(self) -> int:
        bitset = self.sublane.to_sumo_bitset()
        bitset <<= 2
        bitset |= self.respect.to_sumo_bitset()
        bitset <<= 2
        bitset |= self.obligation.to_sumo_bitset()
        bitset <<= 2
        bitset |= self.speed_gain.to_sumo_bitset()
        bitset <<= 2
        bitset |= self.cooperative.to_sumo_bitset()
        bitset <<= 2
        bitset |= self.strategic.to_sumo_bitset()
        return bitset

    @classmethod
    def from_sumo_bitset(cls, bitset: int) -> "SumoLaneChangeMode":
        strategic = SumoLaneChangeReasonConfig.from_sumo_bitset(bitset & 0b11)
        bitset >>= 2
        cooperative = SumoLaneChangeReasonConfig.from_sumo_bitset(bitset & 0b11)
        bitset >>= 2
        speed_gain = SumoLaneChangeReasonConfig.from_sumo_bitset(bitset & 0b11)
        bitset >>= 2
        obligation = SumoLaneChangeReasonConfig.from_sumo_bitset(bitset & 0b11)
        bitset >>= 2
        respect = SumoLaneChangeModeRespect.from_sumo_bitset(bitset & 0b11)
        bitset >>= 2
        sublane = SumoLaneChangeReasonConfig.from_sumo_bitset(bitset & 0b11)
        return SumoLaneChangeMode(
            strategic=strategic,
            cooperative=cooperative,
            speed_gain=speed_gain,
            obligation=obligation,
            respect=respect,
            sublane=sublane,
        )


# Constant used in dataclass metadata fields to define the SUMO parameter name
_SUMO_DEVICE_METADATA_PARAMETER_NAME_KEY = "parameter_name"


@dataclass
class SumoParameterCollection(ABC):
    """
    Base class for an easy to use interface for collections of SUMO parameters. Parameters are key-value pairs, for which no equivialant TraCI command exists and which must be configured trough the generic getParameter/setParameter methods of TraCI.
    """

    @classmethod
    @abstractmethod
    def _get_sumo_parameter_prefix(cls) -> str:
        """
        Defines the prefix that will be added to each parameter key. Usually this is the device name.
        """
        ...

    @classmethod
    def _get_full_parameter_key(cls, parameter_key: str) -> str:
        return f"{cls._get_sumo_parameter_prefix()}.{parameter_key}"

    @classmethod
    def get_parameter_keys(cls) -> Sequence[str]:
        """
        Get a list of all parameter keys that are supported by this collection.
        """
        parameter_keys = []
        for field_item in fields(cls):
            parameter_key = field_item.metadata[
                _SUMO_DEVICE_METADATA_PARAMETER_NAME_KEY
            ]
            parameter_key_full = cls._get_full_parameter_key(parameter_key)
            parameter_keys.append(parameter_key_full)

        return parameter_keys

    def to_sumo_parameter_map(self) -> Mapping[str, Any]:
        """
        Return a mapping of parameter key to parameter value for this parameter collection.
        """
        parameter_map = {}
        for field_item in fields(self):
            parameter_key = field_item.metadata[
                _SUMO_DEVICE_METADATA_PARAMETER_NAME_KEY
            ]
            full_parameter_key = self._get_full_parameter_key(parameter_key)
            parameter_map[full_parameter_key] = self.__getattribute__(field_item.name)
        return parameter_map

    @classmethod
    def from_sumo_parameter_map(
        cls, parameter_map: Mapping[str, Any]
    ) -> "SumoParameterCollection":
        device = cls()

        for field_item in fields(cls):
            parameter_key = field_item.metadata[
                _SUMO_DEVICE_METADATA_PARAMETER_NAME_KEY
            ]
            full_parameter_key = cls._get_full_parameter_key(parameter_key)
            if full_parameter_key not in parameter_map:
                continue

            raw_parameter_value = parameter_map[full_parameter_key]
            if len(raw_parameter_value) == 0:
                # Skip empty values, so the type conversion does not break
                continue

            try:
                # All parameters are of type 'str', therefore we must cast them to their 'real' type
                parameter_value = field_item.type(raw_parameter_value)
            except (ValueError, TypeError) as e:
                raise ValueError(
                    f"Cannot load parameter '{full_parameter_key}' from parameter map: {e}"
                )
            setattr(device, field_item.name, parameter_value)

        return device


def _sumo_parameter_metadata(parameter_name: str) -> Mapping[str, str]:
    """
    Construct the metadata information for a field in a SUMO device.
    """
    return {_SUMO_DEVICE_METADATA_PARAMETER_NAME_KEY: parameter_name}


@dataclass
class SumoDriverState(SumoParameterCollection):
    """
    Induce perception and actuation errors into the simulation by altering the driver state
    https://sumo.dlr.de/docs/Driver_State.html
    """

    @classmethod
    def _get_sumo_parameter_prefix(cls) -> str:
        return "device.driverstate"

    awerness: float = field(
        default=1.0,
        metadata=_sumo_parameter_metadata(parameter_name="initialAwareness"),
    )
    """The initial awareness assigned to the driver state."""

    min_awerness: float = field(
        default=0.1,
        metadata=_sumo_parameter_metadata(parameter_name="minAwareness"),
    )
    """The minimal value for the driver awareness (a technical parameter to avoid a blow up of the term 1/minAwareness)."""

    error_time_scale_coefficient: float = field(
        default=100.0,
        metadata=_sumo_parameter_metadata(parameter_name="errorTimeScaleCoefficient"),
    )
    """Time scale constant that controls the time scale of the underlying error process."""

    error_noice_intensity_coefficient: float = field(
        default=0.2,
        metadata=_sumo_parameter_metadata(
            parameter_name="errorNoiseIntensityCoefficient"
        ),
    )
    """Noise intensity constant that controls the noise intensity of the underlying error process."""

    speed_difference_error_coefficient: float = field(
        default=0.15,
        metadata=_sumo_parameter_metadata(
            parameter_name="speedDifferenceErrorCoefficient"
        ),
    )
    """Scaling coefficient for the error applied to the speed difference input of the car-following model."""

    headway_error_coefficient: float = field(
        default=0.75,
        metadata=_sumo_parameter_metadata(parameter_name="headwayErrorCoefficient"),
    )
    """Scaling coefficient for the error applied to the distance input of the car-following model."""

    # Here 'freeSpeedErrorCoefficient' should be configured, but somehow it is not support through traci...

    speed_difference_change_perception_threshold: float = field(
        default=0.1,
        metadata=_sumo_parameter_metadata(
            parameter_name="speedDifferenceChangePerceptionThreshold"
        ),
    )
    """Constant controlling the threshold for the perception of changes in the speed difference"""

    headway_change_perception_threshold: float = field(
        default=0.1,
        metadata=_sumo_parameter_metadata(
            parameter_name="headwayChangePerceptionThreshold"
        ),
    )
    """Constant controlling the threshold for the perception of changes in the distance input."""


@dataclass
class SumoLaneChangeModel(SumoParameterCollection):
    """
    Abstract representation of the configuration for the lane change models LC2013 and SL2015. Because, both lane change models also have individual configuration options, there exist extra configuration objects for each lane change model.

    For more information see: https://sumo.dlr.de/docs/Definition_of_Vehicles%2C_Vehicle_Types%2C_and_Routes.html#lane-changing_models
    """

    @classmethod
    def _get_sumo_parameter_prefix(cls) -> str:
        return "laneChangeModel"

    strategic: float = field(
        default=1.0, metadata=_sumo_parameter_metadata(parameter_name="lcStrategic")
    )
    """The eagerness for performing strategic lane changing. Higher values result in earlier lane-changing.
    Range [0-inf), with special values: 0 for no lookahead distance, -1 to disable strategic changing completely."""

    cooperative: float = field(
        default=1.0, metadata=_sumo_parameter_metadata(parameter_name="lcCooperative")
    )
    """The willingness for performing cooperative lane changing. Lower values result in reduced cooperation.
    Range [0-1], with special values: 0 permits changing only for higher speed, -1 disables cooperative changing completely."""

    speed_gain: float = field(
        default=1.0, metadata=_sumo_parameter_metadata(parameter_name="lcSpeedGain")
    )
    """The eagerness for performing lane changing to gain speed. Higher values result in more lane-changing.
    Range [0-inf), with 0 disabling lane changing for speed gain."""

    keep_right: float = field(
        default=0.0, metadata=_sumo_parameter_metadata(parameter_name="lcKeepRight")
    )
    """The eagerness for following the obligation to keep right. Higher values result in earlier lane-changing.
    Range [0-inf), with 0 disabling this type of changing."""

    lookahead_left: float = field(
        default=2.0, metadata=_sumo_parameter_metadata(parameter_name="lcLookaheadLeft")
    )
    """Factor for configuring the strategic lookahead distance when a change to the left is necessary (relative to right lookahead).
    Range [0-inf), default is 2.0."""

    max_speed_lat_standing: float = field(
        default=1.0,
        metadata=_sumo_parameter_metadata(parameter_name="lcMaxSpeedLatStanding"),
    )
    """Constant term for lateral speed when standing. Set to 0 to avoid orthogonal sliding.
    Default is maxSpeedLat (i.e., disabled)."""

    max_speed_lat_factor: float = field(
        default=1.0,
        metadata=_sumo_parameter_metadata(parameter_name="lcMaxSpeedLatFactor"),
    )
    """Bound on lateral speed while moving computed as lcMaxSpeedLatStanding + lcMaxSpeedLatFactor * getSpeed().
    If > 0, this is an upper bound (vehicles change slower at low speed), if < 0, this is a lower bound on speed.
    Default is 1.0."""

    sigma: float = field(
        default=0.0, metadata=_sumo_parameter_metadata(parameter_name="lcSigma")
    )
    """Lateral positioning imperfection. Default is 0.0."""


@dataclass
class SumoSL2015LaneChangeModel(SumoLaneChangeModel):
    """
    Configuration for the SL2015 lane change model.
    """

    min_gap_lat: float = field(
        default=0.6, metadata=_sumo_parameter_metadata(parameter_name="minGapLat")
    )
    """The desired minimum lateral gap when using the sublane-model.
    Default is 0.6."""

    sublane: float = field(
        default=1.0, metadata=_sumo_parameter_metadata(parameter_name="lcSublane")
    )
    """The eagerness for using the configured lateral alignment within the lane. Higher values result in increased willingness to sacrifice speed for alignment.
    Range [0-inf), default is 1.0."""

    pushy: float = field(
        default=0.0, metadata=_sumo_parameter_metadata(parameter_name="lcPushy")
    )
    """Willingness to encroach laterally on other drivers.
    Range [0-1], default is 0."""

    pushy_gap: float = field(
        default=0.6, metadata=_sumo_parameter_metadata(parameter_name="lcPushyGap")
    )
    """Minimum lateral gap when encroaching laterally on other drivers (alternative way to define lcPushy).
    Default is minGapLat, range 0 to minGapLat."""

    impatience: float = field(
        default=0.0, metadata=_sumo_parameter_metadata(parameter_name="lcImpatience")
    )
    """Dynamic factor for modifying lcAssertive and lcPushy.
    Default is 0 (no effect), range -1 to 1. At -1 the multiplier is 0.5 and at 1 the multiplier is 1.5."""

    time_to_impatience: float = field(
        default=float("inf"),
        metadata=_sumo_parameter_metadata(parameter_name="lcTimeToImpatience"),
    )
    """Time to reach maximum impatience (of 1). Impatience grows whenever a lane-change maneuver is blocked.
    Default is infinity (disables impatience growth)."""

    accel_lat: float = field(
        default=1.0, metadata=_sumo_parameter_metadata(parameter_name="lcAccelLat")
    )
    """Maximum lateral acceleration per second.
    Default is 1.0."""

    turn_alignment_distance: float = field(
        default=0.0,
        metadata=_sumo_parameter_metadata(parameter_name="lcTurnAlignmentDistance"),
    )
    """Distance to an upcoming turn on the vehicle's route, below which the alignment should be dynamically adapted to match the turn direction.
    Default is 0.0 (disabled)."""

    lane_discipline: float = field(
        default=0.0,
        metadata=_sumo_parameter_metadata(parameter_name="lcLaneDiscipline"),
    )
    """Reluctance to perform speedGain-changes that would place the vehicle across a lane boundary.
    Default is 0.0."""

    max_dist_lat_standing: float = field(
        default=1.6,
        metadata=_sumo_parameter_metadata(parameter_name="lcMaxDistLatStanding"),
    )
    """The maximum lateral maneuver distance in meters while standing (currently used to prevent "sliding" keepRight changes).
    Default is 1.6, 0 for two-wheelers."""


@dataclass
class SumoLC2013LaneChageModel(SumoLaneChangeModel):
    """
    Configuration for the LC2013 lane change model.
    """

    overtake_right: float = field(
        default=1.0, metadata=_sumo_parameter_metadata(parameter_name="lcOvertakeRight")
    )
    """The probability for violating rules against overtaking on the right.
    Range [0-1], with 0 meaning no overtaking on the right."""


@dataclass
class SumoJunctionModel(SumoParameterCollection):
    @classmethod
    def _get_sumo_parameter_prefix(cls) -> str:
        return "junctionModel"

    crossing_gap: float = field(
        default=10.0, metadata=_sumo_parameter_metadata(parameter_name="jmCrossingGap")
    )
    """Minimum distance to pedestrians that are walking towards the conflict point with the ego vehicle.
    If the pedestrians are further away, the vehicle may drive across the pedestrian crossing.
    Lower values make the driver braver (more aggressive). Range: float >= 0 (m), default is 10."""

    ignore_keep_clear_time: float = field(
        default=-1.0,
        metadata=_sumo_parameter_metadata(parameter_name="jmIgnoreKeepClearTime"),
    )
    """Accumulated waiting time after which a vehicle will drive onto an intersection even if it might cause jamming.
    For negative values, the vehicle will always try to keep the junction clear. Default is -1 (disabled)."""

    drive_after_red_time: float = field(
        default=-1.0,
        metadata=_sumo_parameter_metadata(parameter_name="jmDriveAfterRedTime"),
    )
    """Time threshold after which vehicles may violate a red light if the light has recently changed to red.
    When set to 0, vehicles will always drive at yellow but will try to brake at red.
    Default is -1 (disabled), meaning vehicles obey red lights normally."""

    drive_after_yellow_time: float = field(
        default=-1.0,
        metadata=_sumo_parameter_metadata(parameter_name="jmDriveAfterYellowTime"),
    )
    """Time threshold after which vehicles may violate a yellow light if it has recently changed.
    Vehicles too fast to brake always drive at yellow. Default is -1 (disabled), meaning vehicles obey yellow lights normally."""

    drive_red_speed: float = field(
        default=float("inf"),
        metadata=_sumo_parameter_metadata(parameter_name="jmDriveRedSpeed"),
    )
    """Maximum speed for vehicles violating a red light due to drive_after_red_time.
    The given speed will not be exceeded when entering the intersection. Default is maxSpeed."""

    ignore_foe_prob: float = field(
        default=0.0, metadata=_sumo_parameter_metadata(parameter_name="jmIgnoreFoeProb")
    )
    """Probability that vehicles and pedestrians will ignore foe vehicles that have right-of-way.
    The check is performed anew every simulation step. Range: [0,1], default is 0 (foes are not ignored)."""

    ignore_foe_speed: float = field(
        default=0.0,
        metadata=_sumo_parameter_metadata(parameter_name="jmIgnoreFoeSpeed"),
    )
    """Speed threshold used with ignore_foe_prob. Only vehicles with a speed below or equal to this value may be ignored.
    Default is 0 m/s, meaning foes are not ignored unless stopped."""

    ignore_junction_foe_prob: float = field(
        default=0.0,
        metadata=_sumo_parameter_metadata(parameter_name="jmIgnoreJunctionFoeProb"),
    )
    """Probability that vehicles will ignore foes that have already entered a junction.
    The check is performed anew every simulation step. Range: [0,1], default is 0 (foes are not ignored)."""

    sigma_minor: float = field(
        default=0.0, metadata=_sumo_parameter_metadata(parameter_name="jmSigmaMinor")
    )
    """Driving imperfection (dawdling) while passing a minor link after committing to drive and while still on the intersection.
    Default is the value of sigma."""

    stopline_gap: float = field(
        default=1.0, metadata=_sumo_parameter_metadata(parameter_name="jmStoplineGap")
    )
    """Stopping distance in front of a prioritized or traffic light-controlled stop line.
    If the stop line has been relocated by a stopOffset, the maximum of both distances is applied.
    Range: float >= 0 (m), default is 1.0."""

    timegap_minor: float = field(
        default=1.0, metadata=_sumo_parameter_metadata(parameter_name="jmTimegapMinor")
    )
    """Minimum time gap when passing ahead of a prioritized vehicle.
    Default is 1 second."""

    # Here the parameter 'jmStopSignWait' should be, but somehow it is not support by TraCI...

    # The parameter 'impatience' is listed as part of the junction model, but is configured through a pre-definide traci command (vehicletype.setImpatience) and cannot be set/get through the generic parameter methods


@dataclass
class SumoAutomaticRouting(SumoParameterCollection):
    """
    Configure the automatic rerouting device in the SUMO simulation
    https://sumo.dlr.de/docs/Demand/Automatic_Routing.html
    """

    @classmethod
    def _get_sumo_parameter_prefix(cls) -> str:
        return "device.rerouting"

    period: float = field(
        default=0.0, metadata=_sumo_parameter_metadata(parameter_name="period")
    )
    """Individual rerouting period in seconds"""


@dataclass
class SumoGlosa(SumoParameterCollection):
    """Configure Green Light Optimal Speed Advisory (GLOSA) via the glosa device. This allows vehicles to avoid stops at traffic lights when the phase is about to change:
    https://sumo.dlr.de/docs/Simulation/GLOSA.html
    """

    @classmethod
    def _get_sumo_parameter_prefix(cls) -> str:
        return "device.glosa"

    range: float = field(
        default=100.0, metadata=_sumo_parameter_metadata(parameter_name="range")
    )
    """Maximum range from stop line at which glosa functions become active. If the current traffic light also sets this parameter, the minimum value of the device and tls parameter is used."""

    min_speed: float = field(
        default=5.0, metadata=_sumo_parameter_metadata(parameter_name="min-speed")
    )
    """Minimum speed for slow-down maneuver"""

    max_speed_factor: float = field(
        default=1.1, metadata=_sumo_parameter_metadata(parameter_name="max-speedfactor")
    )
    """Maximum speedFactor for trying to reach a green light. Setting a value of 1 will ensure perfect compliance with the speed limit but may still cause slow drivers to speed up."""
