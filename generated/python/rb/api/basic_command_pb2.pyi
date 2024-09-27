from google.protobuf import duration_pb2 as _duration_pb2
from google.protobuf import wrappers_pb2 as _wrappers_pb2
from rb.api import geometry_pb2 as _geometry_pb2
from rb.api import command_header_pb2 as _command_header_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class StopCommand(_message.Message):
    __slots__ = ()
    class Request(_message.Message):
        __slots__ = ("command_header",)
        COMMAND_HEADER_FIELD_NUMBER: _ClassVar[int]
        command_header: _command_header_pb2.CommandHeader.Request
        def __init__(self, command_header: _Optional[_Union[_command_header_pb2.CommandHeader.Request, _Mapping]] = ...) -> None: ...
    class Feedback(_message.Message):
        __slots__ = ("command_header_feedback",)
        COMMAND_HEADER_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        command_header_feedback: _command_header_pb2.CommandHeader.Feedback
        def __init__(self, command_header_feedback: _Optional[_Union[_command_header_pb2.CommandHeader.Feedback, _Mapping]] = ...) -> None: ...
    def __init__(self) -> None: ...

class SE2VelocityCommand(_message.Message):
    __slots__ = ()
    class Request(_message.Message):
        __slots__ = ("command_header", "minimum_time", "velocity", "acceleration_limit")
        COMMAND_HEADER_FIELD_NUMBER: _ClassVar[int]
        MINIMUM_TIME_FIELD_NUMBER: _ClassVar[int]
        VELOCITY_FIELD_NUMBER: _ClassVar[int]
        ACCELERATION_LIMIT_FIELD_NUMBER: _ClassVar[int]
        command_header: _command_header_pb2.CommandHeader.Request
        minimum_time: _duration_pb2.Duration
        velocity: _geometry_pb2.SE2Velocity
        acceleration_limit: _geometry_pb2.SE2Velocity
        def __init__(self, command_header: _Optional[_Union[_command_header_pb2.CommandHeader.Request, _Mapping]] = ..., minimum_time: _Optional[_Union[_duration_pb2.Duration, _Mapping]] = ..., velocity: _Optional[_Union[_geometry_pb2.SE2Velocity, _Mapping]] = ..., acceleration_limit: _Optional[_Union[_geometry_pb2.SE2Velocity, _Mapping]] = ...) -> None: ...
    class Feedback(_message.Message):
        __slots__ = ("command_header_feedback",)
        COMMAND_HEADER_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        command_header_feedback: _command_header_pb2.CommandHeader.Feedback
        def __init__(self, command_header_feedback: _Optional[_Union[_command_header_pb2.CommandHeader.Feedback, _Mapping]] = ...) -> None: ...
    def __init__(self) -> None: ...

class JogCommand(_message.Message):
    __slots__ = ()
    class Request(_message.Message):
        __slots__ = ("command_header", "joint_name", "velocity_limit", "acceleration_limit", "absolute_position", "relative_position", "one_step")
        COMMAND_HEADER_FIELD_NUMBER: _ClassVar[int]
        JOINT_NAME_FIELD_NUMBER: _ClassVar[int]
        VELOCITY_LIMIT_FIELD_NUMBER: _ClassVar[int]
        ACCELERATION_LIMIT_FIELD_NUMBER: _ClassVar[int]
        ABSOLUTE_POSITION_FIELD_NUMBER: _ClassVar[int]
        RELATIVE_POSITION_FIELD_NUMBER: _ClassVar[int]
        ONE_STEP_FIELD_NUMBER: _ClassVar[int]
        command_header: _command_header_pb2.CommandHeader.Request
        joint_name: str
        velocity_limit: _wrappers_pb2.DoubleValue
        acceleration_limit: _wrappers_pb2.DoubleValue
        absolute_position: float
        relative_position: float
        one_step: bool
        def __init__(self, command_header: _Optional[_Union[_command_header_pb2.CommandHeader.Request, _Mapping]] = ..., joint_name: _Optional[str] = ..., velocity_limit: _Optional[_Union[_wrappers_pb2.DoubleValue, _Mapping]] = ..., acceleration_limit: _Optional[_Union[_wrappers_pb2.DoubleValue, _Mapping]] = ..., absolute_position: _Optional[float] = ..., relative_position: _Optional[float] = ..., one_step: bool = ...) -> None: ...
    class Feedback(_message.Message):
        __slots__ = ("command_header_feedback", "target_joint_name")
        COMMAND_HEADER_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        TARGET_JOINT_NAME_FIELD_NUMBER: _ClassVar[int]
        command_header_feedback: _command_header_pb2.CommandHeader.Feedback
        target_joint_name: str
        def __init__(self, command_header_feedback: _Optional[_Union[_command_header_pb2.CommandHeader.Feedback, _Mapping]] = ..., target_joint_name: _Optional[str] = ...) -> None: ...
    def __init__(self) -> None: ...

class JointVelocityCommand(_message.Message):
    __slots__ = ()
    class Request(_message.Message):
        __slots__ = ("command_header", "minimum_time", "velocity", "acceleration_limit")
        COMMAND_HEADER_FIELD_NUMBER: _ClassVar[int]
        MINIMUM_TIME_FIELD_NUMBER: _ClassVar[int]
        VELOCITY_FIELD_NUMBER: _ClassVar[int]
        ACCELERATION_LIMIT_FIELD_NUMBER: _ClassVar[int]
        command_header: _command_header_pb2.CommandHeader.Request
        minimum_time: _duration_pb2.Duration
        velocity: _containers.RepeatedScalarFieldContainer[float]
        acceleration_limit: _containers.RepeatedScalarFieldContainer[float]
        def __init__(self, command_header: _Optional[_Union[_command_header_pb2.CommandHeader.Request, _Mapping]] = ..., minimum_time: _Optional[_Union[_duration_pb2.Duration, _Mapping]] = ..., velocity: _Optional[_Iterable[float]] = ..., acceleration_limit: _Optional[_Iterable[float]] = ...) -> None: ...
    class Feedback(_message.Message):
        __slots__ = ("command_header_feedback",)
        COMMAND_HEADER_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        command_header_feedback: _command_header_pb2.CommandHeader.Feedback
        def __init__(self, command_header_feedback: _Optional[_Union[_command_header_pb2.CommandHeader.Feedback, _Mapping]] = ...) -> None: ...
    def __init__(self) -> None: ...

class JointPositionCommand(_message.Message):
    __slots__ = ()
    class Request(_message.Message):
        __slots__ = ("command_header", "minimum_time", "position", "velocity_limit", "acceleration_limit", "cutoff_frequency")
        COMMAND_HEADER_FIELD_NUMBER: _ClassVar[int]
        MINIMUM_TIME_FIELD_NUMBER: _ClassVar[int]
        POSITION_FIELD_NUMBER: _ClassVar[int]
        VELOCITY_LIMIT_FIELD_NUMBER: _ClassVar[int]
        ACCELERATION_LIMIT_FIELD_NUMBER: _ClassVar[int]
        CUTOFF_FREQUENCY_FIELD_NUMBER: _ClassVar[int]
        command_header: _command_header_pb2.CommandHeader.Request
        minimum_time: _duration_pb2.Duration
        position: _containers.RepeatedScalarFieldContainer[float]
        velocity_limit: _containers.RepeatedScalarFieldContainer[float]
        acceleration_limit: _containers.RepeatedScalarFieldContainer[float]
        cutoff_frequency: _wrappers_pb2.DoubleValue
        def __init__(self, command_header: _Optional[_Union[_command_header_pb2.CommandHeader.Request, _Mapping]] = ..., minimum_time: _Optional[_Union[_duration_pb2.Duration, _Mapping]] = ..., position: _Optional[_Iterable[float]] = ..., velocity_limit: _Optional[_Iterable[float]] = ..., acceleration_limit: _Optional[_Iterable[float]] = ..., cutoff_frequency: _Optional[_Union[_wrappers_pb2.DoubleValue, _Mapping]] = ...) -> None: ...
    class Feedback(_message.Message):
        __slots__ = ("command_header_feedback",)
        COMMAND_HEADER_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        command_header_feedback: _command_header_pb2.CommandHeader.Feedback
        def __init__(self, command_header_feedback: _Optional[_Union[_command_header_pb2.CommandHeader.Feedback, _Mapping]] = ...) -> None: ...
    def __init__(self) -> None: ...

class CartesianCommand(_message.Message):
    __slots__ = ()
    class SE3PoseTarget(_message.Message):
        __slots__ = ("ref_link_name", "link_name", "T", "linear_velocity_limit", "angular_velocity_limit", "acceleration_limit_scaling")
        REF_LINK_NAME_FIELD_NUMBER: _ClassVar[int]
        LINK_NAME_FIELD_NUMBER: _ClassVar[int]
        T_FIELD_NUMBER: _ClassVar[int]
        LINEAR_VELOCITY_LIMIT_FIELD_NUMBER: _ClassVar[int]
        ANGULAR_VELOCITY_LIMIT_FIELD_NUMBER: _ClassVar[int]
        ACCELERATION_LIMIT_SCALING_FIELD_NUMBER: _ClassVar[int]
        ref_link_name: str
        link_name: str
        T: _geometry_pb2.SE3Pose
        linear_velocity_limit: _wrappers_pb2.DoubleValue
        angular_velocity_limit: _wrappers_pb2.DoubleValue
        acceleration_limit_scaling: _wrappers_pb2.DoubleValue
        def __init__(self, ref_link_name: _Optional[str] = ..., link_name: _Optional[str] = ..., T: _Optional[_Union[_geometry_pb2.SE3Pose, _Mapping]] = ..., linear_velocity_limit: _Optional[_Union[_wrappers_pb2.DoubleValue, _Mapping]] = ..., angular_velocity_limit: _Optional[_Union[_wrappers_pb2.DoubleValue, _Mapping]] = ..., acceleration_limit_scaling: _Optional[_Union[_wrappers_pb2.DoubleValue, _Mapping]] = ...) -> None: ...
    class TrackingError(_message.Message):
        __slots__ = ("position_error", "rotation_error")
        POSITION_ERROR_FIELD_NUMBER: _ClassVar[int]
        ROTATION_ERROR_FIELD_NUMBER: _ClassVar[int]
        position_error: float
        rotation_error: float
        def __init__(self, position_error: _Optional[float] = ..., rotation_error: _Optional[float] = ...) -> None: ...
    class Request(_message.Message):
        __slots__ = ("command_header", "minimum_time", "targets", "stop_position_tracking_error", "stop_orientation_tracking_error")
        COMMAND_HEADER_FIELD_NUMBER: _ClassVar[int]
        MINIMUM_TIME_FIELD_NUMBER: _ClassVar[int]
        TARGETS_FIELD_NUMBER: _ClassVar[int]
        STOP_POSITION_TRACKING_ERROR_FIELD_NUMBER: _ClassVar[int]
        STOP_ORIENTATION_TRACKING_ERROR_FIELD_NUMBER: _ClassVar[int]
        command_header: _command_header_pb2.CommandHeader.Request
        minimum_time: _duration_pb2.Duration
        targets: _containers.RepeatedCompositeFieldContainer[CartesianCommand.SE3PoseTarget]
        stop_position_tracking_error: _wrappers_pb2.DoubleValue
        stop_orientation_tracking_error: _wrappers_pb2.DoubleValue
        def __init__(self, command_header: _Optional[_Union[_command_header_pb2.CommandHeader.Request, _Mapping]] = ..., minimum_time: _Optional[_Union[_duration_pb2.Duration, _Mapping]] = ..., targets: _Optional[_Iterable[_Union[CartesianCommand.SE3PoseTarget, _Mapping]]] = ..., stop_position_tracking_error: _Optional[_Union[_wrappers_pb2.DoubleValue, _Mapping]] = ..., stop_orientation_tracking_error: _Optional[_Union[_wrappers_pb2.DoubleValue, _Mapping]] = ...) -> None: ...
    class Feedback(_message.Message):
        __slots__ = ("command_header_feedback", "tracking_errors")
        COMMAND_HEADER_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        TRACKING_ERRORS_FIELD_NUMBER: _ClassVar[int]
        command_header_feedback: _command_header_pb2.CommandHeader.Feedback
        tracking_errors: _containers.RepeatedCompositeFieldContainer[CartesianCommand.TrackingError]
        def __init__(self, command_header_feedback: _Optional[_Union[_command_header_pb2.CommandHeader.Feedback, _Mapping]] = ..., tracking_errors: _Optional[_Iterable[_Union[CartesianCommand.TrackingError, _Mapping]]] = ...) -> None: ...
    def __init__(self) -> None: ...

class GravityCompensationCommand(_message.Message):
    __slots__ = ()
    class Request(_message.Message):
        __slots__ = ("command_header", "on")
        COMMAND_HEADER_FIELD_NUMBER: _ClassVar[int]
        ON_FIELD_NUMBER: _ClassVar[int]
        command_header: _command_header_pb2.CommandHeader.Request
        on: bool
        def __init__(self, command_header: _Optional[_Union[_command_header_pb2.CommandHeader.Request, _Mapping]] = ..., on: bool = ...) -> None: ...
    class Feedback(_message.Message):
        __slots__ = ("command_header_feedback",)
        COMMAND_HEADER_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        command_header_feedback: _command_header_pb2.CommandHeader.Feedback
        def __init__(self, command_header_feedback: _Optional[_Union[_command_header_pb2.CommandHeader.Feedback, _Mapping]] = ...) -> None: ...
    def __init__(self) -> None: ...

class ImpedanceControlCommand(_message.Message):
    __slots__ = ()
    class TrackingError(_message.Message):
        __slots__ = ("position_error", "rotation_error")
        POSITION_ERROR_FIELD_NUMBER: _ClassVar[int]
        ROTATION_ERROR_FIELD_NUMBER: _ClassVar[int]
        position_error: float
        rotation_error: float
        def __init__(self, position_error: _Optional[float] = ..., rotation_error: _Optional[float] = ...) -> None: ...
    class Request(_message.Message):
        __slots__ = ("command_header", "ref_link_name", "link_name", "T", "translation_weight", "rotation_weight")
        COMMAND_HEADER_FIELD_NUMBER: _ClassVar[int]
        REF_LINK_NAME_FIELD_NUMBER: _ClassVar[int]
        LINK_NAME_FIELD_NUMBER: _ClassVar[int]
        T_FIELD_NUMBER: _ClassVar[int]
        TRANSLATION_WEIGHT_FIELD_NUMBER: _ClassVar[int]
        ROTATION_WEIGHT_FIELD_NUMBER: _ClassVar[int]
        command_header: _command_header_pb2.CommandHeader.Request
        ref_link_name: str
        link_name: str
        T: _geometry_pb2.SE3Pose
        translation_weight: _geometry_pb2.Vec3
        rotation_weight: _geometry_pb2.Vec3
        def __init__(self, command_header: _Optional[_Union[_command_header_pb2.CommandHeader.Request, _Mapping]] = ..., ref_link_name: _Optional[str] = ..., link_name: _Optional[str] = ..., T: _Optional[_Union[_geometry_pb2.SE3Pose, _Mapping]] = ..., translation_weight: _Optional[_Union[_geometry_pb2.Vec3, _Mapping]] = ..., rotation_weight: _Optional[_Union[_geometry_pb2.Vec3, _Mapping]] = ...) -> None: ...
    class Feedback(_message.Message):
        __slots__ = ("command_header_feedback", "tracking_error")
        COMMAND_HEADER_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        TRACKING_ERROR_FIELD_NUMBER: _ClassVar[int]
        command_header_feedback: _command_header_pb2.CommandHeader.Feedback
        tracking_error: ImpedanceControlCommand.TrackingError
        def __init__(self, command_header_feedback: _Optional[_Union[_command_header_pb2.CommandHeader.Feedback, _Mapping]] = ..., tracking_error: _Optional[_Union[ImpedanceControlCommand.TrackingError, _Mapping]] = ...) -> None: ...
    def __init__(self) -> None: ...

class OptimalControlCommand(_message.Message):
    __slots__ = ()
    class CartesianCost(_message.Message):
        __slots__ = ("ref_link_name", "link_name", "T", "translation_weight", "rotation_weight")
        REF_LINK_NAME_FIELD_NUMBER: _ClassVar[int]
        LINK_NAME_FIELD_NUMBER: _ClassVar[int]
        T_FIELD_NUMBER: _ClassVar[int]
        TRANSLATION_WEIGHT_FIELD_NUMBER: _ClassVar[int]
        ROTATION_WEIGHT_FIELD_NUMBER: _ClassVar[int]
        ref_link_name: str
        link_name: str
        T: _geometry_pb2.SE3Pose
        translation_weight: float
        rotation_weight: float
        def __init__(self, ref_link_name: _Optional[str] = ..., link_name: _Optional[str] = ..., T: _Optional[_Union[_geometry_pb2.SE3Pose, _Mapping]] = ..., translation_weight: _Optional[float] = ..., rotation_weight: _Optional[float] = ...) -> None: ...
    class CenterOfMassCost(_message.Message):
        __slots__ = ("ref_link_name", "pose", "weight")
        REF_LINK_NAME_FIELD_NUMBER: _ClassVar[int]
        POSE_FIELD_NUMBER: _ClassVar[int]
        WEIGHT_FIELD_NUMBER: _ClassVar[int]
        ref_link_name: str
        pose: _geometry_pb2.Vec3
        weight: float
        def __init__(self, ref_link_name: _Optional[str] = ..., pose: _Optional[_Union[_geometry_pb2.Vec3, _Mapping]] = ..., weight: _Optional[float] = ...) -> None: ...
    class JointPositionCost(_message.Message):
        __slots__ = ("joint_name", "target_position", "weight")
        JOINT_NAME_FIELD_NUMBER: _ClassVar[int]
        TARGET_POSITION_FIELD_NUMBER: _ClassVar[int]
        WEIGHT_FIELD_NUMBER: _ClassVar[int]
        joint_name: str
        target_position: float
        weight: float
        def __init__(self, joint_name: _Optional[str] = ..., target_position: _Optional[float] = ..., weight: _Optional[float] = ...) -> None: ...
    class Request(_message.Message):
        __slots__ = ("command_header", "cartesian_costs", "center_of_mass_cost", "joint_position_costs", "velocity_limit_scaling", "velocity_tracking_gain", "stop_cost", "min_delta_cost", "patience")
        COMMAND_HEADER_FIELD_NUMBER: _ClassVar[int]
        CARTESIAN_COSTS_FIELD_NUMBER: _ClassVar[int]
        CENTER_OF_MASS_COST_FIELD_NUMBER: _ClassVar[int]
        JOINT_POSITION_COSTS_FIELD_NUMBER: _ClassVar[int]
        VELOCITY_LIMIT_SCALING_FIELD_NUMBER: _ClassVar[int]
        VELOCITY_TRACKING_GAIN_FIELD_NUMBER: _ClassVar[int]
        STOP_COST_FIELD_NUMBER: _ClassVar[int]
        MIN_DELTA_COST_FIELD_NUMBER: _ClassVar[int]
        PATIENCE_FIELD_NUMBER: _ClassVar[int]
        command_header: _command_header_pb2.CommandHeader.Request
        cartesian_costs: _containers.RepeatedCompositeFieldContainer[OptimalControlCommand.CartesianCost]
        center_of_mass_cost: OptimalControlCommand.CenterOfMassCost
        joint_position_costs: _containers.RepeatedCompositeFieldContainer[OptimalControlCommand.JointPositionCost]
        velocity_limit_scaling: _wrappers_pb2.DoubleValue
        velocity_tracking_gain: _wrappers_pb2.DoubleValue
        stop_cost: _wrappers_pb2.DoubleValue
        min_delta_cost: _wrappers_pb2.DoubleValue
        patience: _wrappers_pb2.Int32Value
        def __init__(self, command_header: _Optional[_Union[_command_header_pb2.CommandHeader.Request, _Mapping]] = ..., cartesian_costs: _Optional[_Iterable[_Union[OptimalControlCommand.CartesianCost, _Mapping]]] = ..., center_of_mass_cost: _Optional[_Union[OptimalControlCommand.CenterOfMassCost, _Mapping]] = ..., joint_position_costs: _Optional[_Iterable[_Union[OptimalControlCommand.JointPositionCost, _Mapping]]] = ..., velocity_limit_scaling: _Optional[_Union[_wrappers_pb2.DoubleValue, _Mapping]] = ..., velocity_tracking_gain: _Optional[_Union[_wrappers_pb2.DoubleValue, _Mapping]] = ..., stop_cost: _Optional[_Union[_wrappers_pb2.DoubleValue, _Mapping]] = ..., min_delta_cost: _Optional[_Union[_wrappers_pb2.DoubleValue, _Mapping]] = ..., patience: _Optional[_Union[_wrappers_pb2.Int32Value, _Mapping]] = ...) -> None: ...
    class Feedback(_message.Message):
        __slots__ = ("command_header_feedback", "total_cost", "cartesian_costs", "center_of_mass_cost", "joint_position_costs")
        COMMAND_HEADER_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        TOTAL_COST_FIELD_NUMBER: _ClassVar[int]
        CARTESIAN_COSTS_FIELD_NUMBER: _ClassVar[int]
        CENTER_OF_MASS_COST_FIELD_NUMBER: _ClassVar[int]
        JOINT_POSITION_COSTS_FIELD_NUMBER: _ClassVar[int]
        command_header_feedback: _command_header_pb2.CommandHeader.Feedback
        total_cost: float
        cartesian_costs: _containers.RepeatedScalarFieldContainer[float]
        center_of_mass_cost: float
        joint_position_costs: _containers.RepeatedScalarFieldContainer[float]
        def __init__(self, command_header_feedback: _Optional[_Union[_command_header_pb2.CommandHeader.Feedback, _Mapping]] = ..., total_cost: _Optional[float] = ..., cartesian_costs: _Optional[_Iterable[float]] = ..., center_of_mass_cost: _Optional[float] = ..., joint_position_costs: _Optional[_Iterable[float]] = ...) -> None: ...
    def __init__(self) -> None: ...

class RealTimeControlCommand(_message.Message):
    __slots__ = ()
    class Request(_message.Message):
        __slots__ = ("command_header", "port")
        COMMAND_HEADER_FIELD_NUMBER: _ClassVar[int]
        PORT_FIELD_NUMBER: _ClassVar[int]
        command_header: _command_header_pb2.CommandHeader.Request
        port: int
        def __init__(self, command_header: _Optional[_Union[_command_header_pb2.CommandHeader.Request, _Mapping]] = ..., port: _Optional[int] = ...) -> None: ...
    class Feedback(_message.Message):
        __slots__ = ("command_header_feedback",)
        COMMAND_HEADER_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        command_header_feedback: _command_header_pb2.CommandHeader.Feedback
        def __init__(self, command_header_feedback: _Optional[_Union[_command_header_pb2.CommandHeader.Feedback, _Mapping]] = ...) -> None: ...
    def __init__(self) -> None: ...
