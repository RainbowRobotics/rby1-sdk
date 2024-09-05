from dataclasses import dataclass, field
import numpy as np
import numpy.typing as npt
from typing import Optional

from ._bindings import *


@dataclass
class CommandHeader:
    control_hold_time: Optional[float] = field(default=None)

    def build(self):
        builder = CommandHeaderBuilder()
        if self.control_hold_time is not None:
            builder.set_control_hold_time(self.control_hold_time)
        return builder


@dataclass
class JointPositionCommand:
    command_header: Optional[CommandHeader] = field(default=None)
    minimum_time: Optional[float] = field(default=None)
    position: Optional[npt.ArrayLike] = field(default=None)
    velocity: Optional[npt.ArrayLike] = field(default=None)
    acceleration_limit: Optional[npt.ArrayLike] = field(default=None)

    def build(self):
        builder = JointPositionCommandBuilder()
        if self.command_header is not None:
            builder.set_command_header(self.command_header)
        if self.minimum_time is not None:
            builder.set_minimum_time(self.minimum_time)
        if self.position is not None:
            builder.set_position(self.position)
        if self.velocity is not None:
            builder.set_velocity(self.velocity)
        if self.acceleration_limit is not None:
            builder.set_acceleration_limit(self.acceleration_limit)
        return builder


@dataclass
class OptimalControlCommand:
    @dataclass
    class CartesianTarget:
        ref_link_name: str
        link_name: str
        T: npt.ArrayLike
        translation_weight: float
        rotation_weight: float

    @dataclass
    class CenterOfMassTarget:
        ref_link_name: str
        pose: npt.ArrayLike
        weight: float

    @dataclass
    class JointPositionTarget:
        joint_name: str
        target_position: float
        weight: float

    command_header: Optional[CommandHeader] = field(default=None)
    cartesian_targets: Optional[list[CartesianTarget]] = field(default=None)
    center_of_mass_target: Optional[CenterOfMassTarget] = field(default=None)
    joint_position_targets: Optional[list[JointPositionTarget]] = field(default=None)
    velocity_limit_scaling: Optional[float] = field(default=None)
    velocity_tracking_gain: Optional[float] = field(default=None)
    stop_cost: Optional[float] = field(default=None)
    min_delta_cost: Optional[float] = field(default=None)
    patience: Optional[int] = field(default=None)

    def build(self):
        builder = OptimalControlCommandBuilder()
        if self.command_header is not None:
            builder.set_command_header(self.command_header)
        if self.cartesian_targets is not None:
            for target in self.cartesian_targets:
                builder.add_cartesian_target(target.ref_link_name, target.link_name, target.T,
                                             target.translation_weight, target.rotation_weight)
        if self.center_of_mass_target is not None:
            builder.set_center_of_mass_target(self.center_of_mass_target)
        if self.joint_position_targets is not None:
            for target in self.joint_position_targets:
                builder.add_joint_position_target(target.joint_name, target.target_position, target.weight)
        if self.velocity_limit_scaling is not None:
            builder.set_velocity_limit_scaling(self.velocity_limit_scaling)
        if self.velocity_tracking_gain is not None:
            builder.set_velocity_tracking_gain(self.velocity_tracking_gain)
        if self.stop_cost is not None:
            builder.set_stop_cost(self.stop_cost)
        if self.min_delta_cost is not None:
            builder.set_min_delta_cost(self.min_delta_cost)
        if self.patience is not None:
            builder.set_patience(self.patience)
        return builder
