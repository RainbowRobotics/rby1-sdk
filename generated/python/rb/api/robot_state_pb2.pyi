from google.protobuf import timestamp_pb2 as _timestamp_pb2
from google.protobuf import duration_pb2 as _duration_pb2
from rb.api import header_pb2 as _header_pb2
from rb.api import geometry_pb2 as _geometry_pb2
from rb.api import control_manager_pb2 as _control_manager_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class SystemStat(_message.Message):
    __slots__ = ("cpu_usage", "memory_usage", "uptime", "program_uptime")
    CPU_USAGE_FIELD_NUMBER: _ClassVar[int]
    MEMORY_USAGE_FIELD_NUMBER: _ClassVar[int]
    UPTIME_FIELD_NUMBER: _ClassVar[int]
    PROGRAM_UPTIME_FIELD_NUMBER: _ClassVar[int]
    cpu_usage: float
    memory_usage: float
    uptime: float
    program_uptime: float
    def __init__(self, cpu_usage: _Optional[float] = ..., memory_usage: _Optional[float] = ..., uptime: _Optional[float] = ..., program_uptime: _Optional[float] = ...) -> None: ...

class BatteryState(_message.Message):
    __slots__ = ("voltage", "current", "level_percent")
    VOLTAGE_FIELD_NUMBER: _ClassVar[int]
    CURRENT_FIELD_NUMBER: _ClassVar[int]
    LEVEL_PERCENT_FIELD_NUMBER: _ClassVar[int]
    voltage: float
    current: float
    level_percent: float
    def __init__(self, voltage: _Optional[float] = ..., current: _Optional[float] = ..., level_percent: _Optional[float] = ...) -> None: ...

class PowerState(_message.Message):
    __slots__ = ("state", "voltage")
    class State(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        STATE_UNKNOWN: _ClassVar[PowerState.State]
        STATE_POWER_ON: _ClassVar[PowerState.State]
        STATE_POWER_OFF: _ClassVar[PowerState.State]
    STATE_UNKNOWN: PowerState.State
    STATE_POWER_ON: PowerState.State
    STATE_POWER_OFF: PowerState.State
    STATE_FIELD_NUMBER: _ClassVar[int]
    VOLTAGE_FIELD_NUMBER: _ClassVar[int]
    state: PowerState.State
    voltage: float
    def __init__(self, state: _Optional[_Union[PowerState.State, str]] = ..., voltage: _Optional[float] = ...) -> None: ...

class EMOState(_message.Message):
    __slots__ = ("state",)
    class State(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        STATE_RELEASED: _ClassVar[EMOState.State]
        STATE_PRESSED: _ClassVar[EMOState.State]
    STATE_RELEASED: EMOState.State
    STATE_PRESSED: EMOState.State
    STATE_FIELD_NUMBER: _ClassVar[int]
    state: EMOState.State
    def __init__(self, state: _Optional[_Union[EMOState.State, str]] = ...) -> None: ...

class JointState(_message.Message):
    __slots__ = ("is_ready", "fet_state", "run_state", "init_state", "motor_type", "motor_state", "time_since_last_update", "power_on", "position", "velocity", "current", "torque", "target_position", "target_velocity", "target_feedback_gain", "target_feedforward_torque", "temperature")
    class FETState(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        FET_STATE_UNKNOWN: _ClassVar[JointState.FETState]
        FET_STATE_ON: _ClassVar[JointState.FETState]
        FET_STATE_OFF: _ClassVar[JointState.FETState]
    FET_STATE_UNKNOWN: JointState.FETState
    FET_STATE_ON: JointState.FETState
    FET_STATE_OFF: JointState.FETState
    class RunState(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        RUN_STATE_UNKNOWN: _ClassVar[JointState.RunState]
        RUN_STATE_CONTROL_ON: _ClassVar[JointState.RunState]
        RUN_STATE_CONTROL_OFF: _ClassVar[JointState.RunState]
    RUN_STATE_UNKNOWN: JointState.RunState
    RUN_STATE_CONTROL_ON: JointState.RunState
    RUN_STATE_CONTROL_OFF: JointState.RunState
    class InitializationState(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        INIT_STATE_UNKNOWN: _ClassVar[JointState.InitializationState]
        INIT_STATE_INITIALIZED: _ClassVar[JointState.InitializationState]
        INIT_STATE_UNINITIALIZED: _ClassVar[JointState.InitializationState]
    INIT_STATE_UNKNOWN: JointState.InitializationState
    INIT_STATE_INITIALIZED: JointState.InitializationState
    INIT_STATE_UNINITIALIZED: JointState.InitializationState
    IS_READY_FIELD_NUMBER: _ClassVar[int]
    FET_STATE_FIELD_NUMBER: _ClassVar[int]
    RUN_STATE_FIELD_NUMBER: _ClassVar[int]
    INIT_STATE_FIELD_NUMBER: _ClassVar[int]
    MOTOR_TYPE_FIELD_NUMBER: _ClassVar[int]
    MOTOR_STATE_FIELD_NUMBER: _ClassVar[int]
    TIME_SINCE_LAST_UPDATE_FIELD_NUMBER: _ClassVar[int]
    POWER_ON_FIELD_NUMBER: _ClassVar[int]
    POSITION_FIELD_NUMBER: _ClassVar[int]
    VELOCITY_FIELD_NUMBER: _ClassVar[int]
    CURRENT_FIELD_NUMBER: _ClassVar[int]
    TORQUE_FIELD_NUMBER: _ClassVar[int]
    TARGET_POSITION_FIELD_NUMBER: _ClassVar[int]
    TARGET_VELOCITY_FIELD_NUMBER: _ClassVar[int]
    TARGET_FEEDBACK_GAIN_FIELD_NUMBER: _ClassVar[int]
    TARGET_FEEDFORWARD_TORQUE_FIELD_NUMBER: _ClassVar[int]
    TEMPERATURE_FIELD_NUMBER: _ClassVar[int]
    is_ready: bool
    fet_state: JointState.FETState
    run_state: JointState.RunState
    init_state: JointState.InitializationState
    motor_type: int
    motor_state: int
    time_since_last_update: _duration_pb2.Duration
    power_on: bool
    position: float
    velocity: float
    current: float
    torque: float
    target_position: float
    target_velocity: float
    target_feedback_gain: int
    target_feedforward_torque: float
    temperature: int
    def __init__(self, is_ready: bool = ..., fet_state: _Optional[_Union[JointState.FETState, str]] = ..., run_state: _Optional[_Union[JointState.RunState, str]] = ..., init_state: _Optional[_Union[JointState.InitializationState, str]] = ..., motor_type: _Optional[int] = ..., motor_state: _Optional[int] = ..., time_since_last_update: _Optional[_Union[_duration_pb2.Duration, _Mapping]] = ..., power_on: bool = ..., position: _Optional[float] = ..., velocity: _Optional[float] = ..., current: _Optional[float] = ..., torque: _Optional[float] = ..., target_position: _Optional[float] = ..., target_velocity: _Optional[float] = ..., target_feedback_gain: _Optional[int] = ..., target_feedforward_torque: _Optional[float] = ..., temperature: _Optional[int] = ...) -> None: ...

class ToolFlangeState(_message.Message):
    __slots__ = ("time_since_last_update", "gyro", "acceleration", "switch_A", "output_voltage")
    TIME_SINCE_LAST_UPDATE_FIELD_NUMBER: _ClassVar[int]
    GYRO_FIELD_NUMBER: _ClassVar[int]
    ACCELERATION_FIELD_NUMBER: _ClassVar[int]
    SWITCH_A_FIELD_NUMBER: _ClassVar[int]
    OUTPUT_VOLTAGE_FIELD_NUMBER: _ClassVar[int]
    time_since_last_update: _duration_pb2.Duration
    gyro: _geometry_pb2.Vec3
    acceleration: _geometry_pb2.Vec3
    switch_A: bool
    output_voltage: int
    def __init__(self, time_since_last_update: _Optional[_Union[_duration_pb2.Duration, _Mapping]] = ..., gyro: _Optional[_Union[_geometry_pb2.Vec3, _Mapping]] = ..., acceleration: _Optional[_Union[_geometry_pb2.Vec3, _Mapping]] = ..., switch_A: bool = ..., output_voltage: _Optional[int] = ...) -> None: ...

class FTSensorData(_message.Message):
    __slots__ = ("time_since_last_update", "force", "torque")
    TIME_SINCE_LAST_UPDATE_FIELD_NUMBER: _ClassVar[int]
    FORCE_FIELD_NUMBER: _ClassVar[int]
    TORQUE_FIELD_NUMBER: _ClassVar[int]
    time_since_last_update: _duration_pb2.Duration
    force: _geometry_pb2.Vec3
    torque: _geometry_pb2.Vec3
    def __init__(self, time_since_last_update: _Optional[_Union[_duration_pb2.Duration, _Mapping]] = ..., force: _Optional[_Union[_geometry_pb2.Vec3, _Mapping]] = ..., torque: _Optional[_Union[_geometry_pb2.Vec3, _Mapping]] = ...) -> None: ...

class Collision(_message.Message):
    __slots__ = ("link1", "link2", "position1", "position2", "distance")
    LINK1_FIELD_NUMBER: _ClassVar[int]
    LINK2_FIELD_NUMBER: _ClassVar[int]
    POSITION1_FIELD_NUMBER: _ClassVar[int]
    POSITION2_FIELD_NUMBER: _ClassVar[int]
    DISTANCE_FIELD_NUMBER: _ClassVar[int]
    link1: str
    link2: str
    position1: _geometry_pb2.Vec3
    position2: _geometry_pb2.Vec3
    distance: float
    def __init__(self, link1: _Optional[str] = ..., link2: _Optional[str] = ..., position1: _Optional[_Union[_geometry_pb2.Vec3, _Mapping]] = ..., position2: _Optional[_Union[_geometry_pb2.Vec3, _Mapping]] = ..., distance: _Optional[float] = ...) -> None: ...

class RobotState(_message.Message):
    __slots__ = ("timestamp", "system_stat", "battery_state", "power_states", "emo_states", "joint_states", "tool_flange_right", "tool_flange_left", "ft_sensor_right", "ft_sensor_left", "is_ready", "position", "velocity", "current", "torque", "target_position", "target_velocity", "target_feedback_gain", "target_feedforward_torque", "odometry", "center_of_mass", "collisions", "temperature")
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    SYSTEM_STAT_FIELD_NUMBER: _ClassVar[int]
    BATTERY_STATE_FIELD_NUMBER: _ClassVar[int]
    POWER_STATES_FIELD_NUMBER: _ClassVar[int]
    EMO_STATES_FIELD_NUMBER: _ClassVar[int]
    JOINT_STATES_FIELD_NUMBER: _ClassVar[int]
    TOOL_FLANGE_RIGHT_FIELD_NUMBER: _ClassVar[int]
    TOOL_FLANGE_LEFT_FIELD_NUMBER: _ClassVar[int]
    FT_SENSOR_RIGHT_FIELD_NUMBER: _ClassVar[int]
    FT_SENSOR_LEFT_FIELD_NUMBER: _ClassVar[int]
    IS_READY_FIELD_NUMBER: _ClassVar[int]
    POSITION_FIELD_NUMBER: _ClassVar[int]
    VELOCITY_FIELD_NUMBER: _ClassVar[int]
    CURRENT_FIELD_NUMBER: _ClassVar[int]
    TORQUE_FIELD_NUMBER: _ClassVar[int]
    TARGET_POSITION_FIELD_NUMBER: _ClassVar[int]
    TARGET_VELOCITY_FIELD_NUMBER: _ClassVar[int]
    TARGET_FEEDBACK_GAIN_FIELD_NUMBER: _ClassVar[int]
    TARGET_FEEDFORWARD_TORQUE_FIELD_NUMBER: _ClassVar[int]
    ODOMETRY_FIELD_NUMBER: _ClassVar[int]
    CENTER_OF_MASS_FIELD_NUMBER: _ClassVar[int]
    COLLISIONS_FIELD_NUMBER: _ClassVar[int]
    TEMPERATURE_FIELD_NUMBER: _ClassVar[int]
    timestamp: _timestamp_pb2.Timestamp
    system_stat: SystemStat
    battery_state: BatteryState
    power_states: _containers.RepeatedCompositeFieldContainer[PowerState]
    emo_states: _containers.RepeatedCompositeFieldContainer[EMOState]
    joint_states: _containers.RepeatedCompositeFieldContainer[JointState]
    tool_flange_right: ToolFlangeState
    tool_flange_left: ToolFlangeState
    ft_sensor_right: FTSensorData
    ft_sensor_left: FTSensorData
    is_ready: _containers.RepeatedScalarFieldContainer[float]
    position: _containers.RepeatedScalarFieldContainer[float]
    velocity: _containers.RepeatedScalarFieldContainer[float]
    current: _containers.RepeatedScalarFieldContainer[float]
    torque: _containers.RepeatedScalarFieldContainer[float]
    target_position: _containers.RepeatedScalarFieldContainer[float]
    target_velocity: _containers.RepeatedScalarFieldContainer[float]
    target_feedback_gain: _containers.RepeatedScalarFieldContainer[int]
    target_feedforward_torque: _containers.RepeatedScalarFieldContainer[float]
    odometry: _geometry_pb2.SE2Pose
    center_of_mass: _geometry_pb2.Vec3
    collisions: _containers.RepeatedCompositeFieldContainer[Collision]
    temperature: _containers.RepeatedScalarFieldContainer[int]
    def __init__(self, timestamp: _Optional[_Union[_timestamp_pb2.Timestamp, _Mapping]] = ..., system_stat: _Optional[_Union[SystemStat, _Mapping]] = ..., battery_state: _Optional[_Union[BatteryState, _Mapping]] = ..., power_states: _Optional[_Iterable[_Union[PowerState, _Mapping]]] = ..., emo_states: _Optional[_Iterable[_Union[EMOState, _Mapping]]] = ..., joint_states: _Optional[_Iterable[_Union[JointState, _Mapping]]] = ..., tool_flange_right: _Optional[_Union[ToolFlangeState, _Mapping]] = ..., tool_flange_left: _Optional[_Union[ToolFlangeState, _Mapping]] = ..., ft_sensor_right: _Optional[_Union[FTSensorData, _Mapping]] = ..., ft_sensor_left: _Optional[_Union[FTSensorData, _Mapping]] = ..., is_ready: _Optional[_Iterable[float]] = ..., position: _Optional[_Iterable[float]] = ..., velocity: _Optional[_Iterable[float]] = ..., current: _Optional[_Iterable[float]] = ..., torque: _Optional[_Iterable[float]] = ..., target_position: _Optional[_Iterable[float]] = ..., target_velocity: _Optional[_Iterable[float]] = ..., target_feedback_gain: _Optional[_Iterable[int]] = ..., target_feedforward_torque: _Optional[_Iterable[float]] = ..., odometry: _Optional[_Union[_geometry_pb2.SE2Pose, _Mapping]] = ..., center_of_mass: _Optional[_Union[_geometry_pb2.Vec3, _Mapping]] = ..., collisions: _Optional[_Iterable[_Union[Collision, _Mapping]]] = ..., temperature: _Optional[_Iterable[int]] = ...) -> None: ...

class GetRobotStateRequest(_message.Message):
    __slots__ = ("request_header",)
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ...) -> None: ...

class GetRobotStateResponse(_message.Message):
    __slots__ = ("response_header", "robot_state", "control_manager_state")
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    ROBOT_STATE_FIELD_NUMBER: _ClassVar[int]
    CONTROL_MANAGER_STATE_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    robot_state: RobotState
    control_manager_state: _control_manager_pb2.ControlManagerState
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., robot_state: _Optional[_Union[RobotState, _Mapping]] = ..., control_manager_state: _Optional[_Union[_control_manager_pb2.ControlManagerState, _Mapping]] = ...) -> None: ...

class GetRobotStateStreamRequest(_message.Message):
    __slots__ = ("request_header", "update_rate")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    UPDATE_RATE_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    update_rate: float
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., update_rate: _Optional[float] = ...) -> None: ...

class GetRobotStateStreamResponse(_message.Message):
    __slots__ = ("response_header", "robot_state", "control_manager_state")
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    ROBOT_STATE_FIELD_NUMBER: _ClassVar[int]
    CONTROL_MANAGER_STATE_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    robot_state: RobotState
    control_manager_state: _control_manager_pb2.ControlManagerState
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., robot_state: _Optional[_Union[RobotState, _Mapping]] = ..., control_manager_state: _Optional[_Union[_control_manager_pb2.ControlManagerState, _Mapping]] = ...) -> None: ...

class ResetOdometryRequest(_message.Message):
    __slots__ = ("request_header", "initial_pose")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    INITIAL_POSE_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    initial_pose: _geometry_pb2.SE2Pose
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., initial_pose: _Optional[_Union[_geometry_pb2.SE2Pose, _Mapping]] = ...) -> None: ...

class ResetOdometryResponse(_message.Message):
    __slots__ = ("response_header",)
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ...) -> None: ...

class GetControlManagerStateRequest(_message.Message):
    __slots__ = ("request_header",)
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ...) -> None: ...

class GetControlManagerStateResponse(_message.Message):
    __slots__ = ("response_header", "control_manager_state")
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    CONTROL_MANAGER_STATE_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    control_manager_state: _control_manager_pb2.ControlManagerState
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., control_manager_state: _Optional[_Union[_control_manager_pb2.ControlManagerState, _Mapping]] = ...) -> None: ...
