from abc import ABC, abstractmethod
from copy import deepcopy
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np
from commonroad.common.util import Interval
from commonroad.scenario.obstacle import ObstacleType, Rectangle
from typing_extensions import override


class DrivingModelParametersProvider(ABC):
    """
    Provides parameters based on the given obstacle type
    """

    @abstractmethod
    def get_shape(self, obstacle_type: ObstacleType) -> Rectangle: ...

    @abstractmethod
    def get_accel(self, obstacle_type: ObstacleType) -> float: ...

    @abstractmethod
    def get_decel(self, obstacle_type: ObstacleType) -> float: ...

    @abstractmethod
    def get_min_gap(self, obstacle_type: ObstacleType) -> float: ...

    @abstractmethod
    def get_max_speed(self, obstacle_type: ObstacleType) -> float: ...

    @abstractmethod
    def get_generic_parameter(
        self, obstacle_type: ObstacleType, param_name: str
    ) -> float: ...


@dataclass
class DrivingModelParameters:
    length: float | Interval
    width: float | Interval
    min_gap: float | Interval

    accel: float | Interval | None = None
    """Maximum allowed acceleration."""

    decel: float | Interval | None = None
    """Maximum allowed deceleration."""

    max_speed: float | Interval | None = None
    """Maximum speed."""

    lc_strategic: float | Interval | None = Interval(10, 100)
    """Eagerness for performing strategic lane changing. Higher values result in earlier lane-changing."""

    lc_speed_gain: float | Interval | None = Interval(3, 20)
    """Eagerness for performing lane changing to gain speed. Higher values result in more lane-changing."""

    lc_cooperative: float | Interval | None = Interval(1, 3)
    """Willingness for performing cooperative lane changing. Lower values result in reduced cooperation."""

    lc_sigma: float | Interval | None = Interval(0.1, 0.2)
    sigma: float | Interval | None = Interval(0.5, 0.65)
    """Driver imperfection. 0 denotes perfect driving."""

    lc_impatience: float | Interval | None = Interval(0, 0.5)

    impatience: float | Interval | None = Interval(0, 0.5)
    speed_dev: float | Interval | None = Interval(0.1, 0.2)
    speed_factor: float | Interval | None = Interval(0.9, 1.1)


class StaticDrivingModelParametersProvider(DrivingModelParametersProvider):
    """
    Provides a parameter sampling over a static, predefined set of values.
    If the parameter is a simple value this value will be used.
    If the parameter is defined as an interval, values will be selected from this interval according to a uniform distribution.
    """

    DEFAULT_DRIVING_MODEL_PARAMETERS = {
        ObstacleType.CAR: DrivingModelParameters(
            length=5.0,
            width=2.0,
            # default 2.9 m/s²
            accel=Interval(2, 2.9),
            # default 7.5 m/s²
            decel=Interval(4, 6.5),
            # default 180/3.6 m/s
            max_speed=180 / 3.6,
            min_gap=2.5,
        ),
        ObstacleType.TRUCK: DrivingModelParameters(
            length=7.5,
            width=2.6,
            accel=Interval(1, 1.5),
            decel=Interval(3, 4.5),
            max_speed=130 / 3.6,
            min_gap=2.5,
        ),
        ObstacleType.BUS: DrivingModelParameters(
            length=12.4,
            width=2.7,
            min_gap=2.5,
            accel=Interval(1, 1.4),
            decel=Interval(3, 4.5),
            max_speed=85 / 3.6,
        ),
        ObstacleType.BICYCLE: DrivingModelParameters(
            length=2.0,
            width=0.68,
            # default 0.5
            min_gap=1.0,
            # default 1.2
            accel=Interval(1, 1.4),
            # default 3
            decel=Interval(2.5, 3.5),
            # default 85/3.6
            max_speed=25 / 3.6,
        ),
        ObstacleType.PEDESTRIAN: DrivingModelParameters(
            length=0.415, width=0.678, min_gap=0.25
        ),
    }

    def __init__(
        self,
        driving_model_parameters: Optional[
            Dict[ObstacleType, DrivingModelParameters]
        ] = None,
    ):
        # We need to copy the parameter dict here, because otherwise overriding the values
        # would also override the values for all other instances
        self._driving_model_parameters = deepcopy(self.DEFAULT_DRIVING_MODEL_PARAMETERS)
        if driving_model_parameters:
            # If we received model parameters, those should override the default ones
            for obstacle_type, parameters_set in driving_model_parameters.items():
                self._driving_model_parameters[obstacle_type] = parameters_set

    def get_driving_model_parameters(
        self, obstacle_type: ObstacleType
    ) -> DrivingModelParameters:
        if obstacle_type in self._driving_model_parameters:
            return self._driving_model_parameters[obstacle_type]
        else:
            raise ValueError(
                f"No driving model parameters for obstacle type '{obstacle_type}'"
            )

    def sample_driving_model_parameter(self, parameter: Interval | float):
        if isinstance(parameter, Interval):
            assert (
                0 <= parameter.start <= parameter.end
            ), f"All values in the interval need to be positive: {parameter}"
            return float(np.random.uniform(parameter.start, parameter.end))
        else:
            return parameter

    @override
    def get_shape(self, obstacle_type: ObstacleType) -> Rectangle:
        """
        :return: A Rectangle with the length and width sampled
        """
        # TODO: The legacy implementation only allowed the definition of Rectangles
        # but it will be great if the caller could provide the base shape
        parameters = self.get_driving_model_parameters(obstacle_type)
        length = self.sample_driving_model_parameter(parameters.length)
        width = self.sample_driving_model_parameter(parameters.width)
        return Rectangle(length, width)

    @override
    def get_accel(self, obstacle_type: ObstacleType) -> float:
        parameters = self.get_driving_model_parameters(obstacle_type)
        if parameters.accel is None:
            raise ValueError(
                f"Parameter 'accel' is not set for obstacle type '{obstacle_type}'"
            )
        return self.sample_driving_model_parameter(parameters.accel)

    @override
    def get_decel(self, obstacle_type: ObstacleType) -> float:
        parameters = self.get_driving_model_parameters(obstacle_type)
        if parameters.decel is None:
            raise ValueError(
                f"Parameter 'decel' is not set for obstacle type '{obstacle_type}'"
            )
        return self.sample_driving_model_parameter(parameters.decel)

    @override
    def get_min_gap(self, obstacle_type: ObstacleType) -> float:
        parameters = self.get_driving_model_parameters(obstacle_type)
        return self.sample_driving_model_parameter(parameters.min_gap)

    @override
    def get_max_speed(self, obstacle_type: ObstacleType) -> float:
        parameters = self.get_driving_model_parameters(obstacle_type)
        if parameters.max_speed is None:
            raise ValueError(
                f"Parameter 'max_speed' is not set for obstacle type '{obstacle_type}'"
            )
        return self.sample_driving_model_parameter(parameters.max_speed)

    @override
    def get_generic_parameter(
        self,
        obstacle_type: ObstacleType,
        param_name: str,
    ) -> float:
        parameters = self.get_driving_model_parameters(obstacle_type)
        if not hasattr(parameters, param_name):
            raise ValueError(
                f"Parameter '{param_name}' is not set for obstacle type '{obstacle_type}'"
            )

        return self.sample_driving_model_parameter(getattr(parameters, param_name))
