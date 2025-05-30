from rb.api import header_pb2 as _header_pb2
from google.protobuf import wrappers_pb2 as _wrappers_pb2
from google.protobuf import duration_pb2 as _duration_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class ControlManagerState(_message.Message):
    __slots__ = ("state", "time_scale", "control_state", "enabled_joint_idx", "unlimited_mode_enabled")
    class State(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        CONTROL_MANAGER_STATE_UNKNOWN: _ClassVar[ControlManagerState.State]
        CONTROL_MANAGER_STATE_IDLE: _ClassVar[ControlManagerState.State]
        CONTROL_MANAGER_STATE_ENABLED: _ClassVar[ControlManagerState.State]
        CONTROL_MANAGER_STATE_MINOR_FAULT: _ClassVar[ControlManagerState.State]
        CONTROL_MANAGER_STATE_MAJOR_FAULT: _ClassVar[ControlManagerState.State]
    CONTROL_MANAGER_STATE_UNKNOWN: ControlManagerState.State
    CONTROL_MANAGER_STATE_IDLE: ControlManagerState.State
    CONTROL_MANAGER_STATE_ENABLED: ControlManagerState.State
    CONTROL_MANAGER_STATE_MINOR_FAULT: ControlManagerState.State
    CONTROL_MANAGER_STATE_MAJOR_FAULT: ControlManagerState.State
    class ControlState(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        CONTROL_STATE_UNKNOWN: _ClassVar[ControlManagerState.ControlState]
        CONTROL_STATE_IDLE: _ClassVar[ControlManagerState.ControlState]
        CONTROL_STATE_EXECUTING: _ClassVar[ControlManagerState.ControlState]
        CONTROL_STATE_SWITCHING: _ClassVar[ControlManagerState.ControlState]
    CONTROL_STATE_UNKNOWN: ControlManagerState.ControlState
    CONTROL_STATE_IDLE: ControlManagerState.ControlState
    CONTROL_STATE_EXECUTING: ControlManagerState.ControlState
    CONTROL_STATE_SWITCHING: ControlManagerState.ControlState
    STATE_FIELD_NUMBER: _ClassVar[int]
    TIME_SCALE_FIELD_NUMBER: _ClassVar[int]
    CONTROL_STATE_FIELD_NUMBER: _ClassVar[int]
    ENABLED_JOINT_IDX_FIELD_NUMBER: _ClassVar[int]
    UNLIMITED_MODE_ENABLED_FIELD_NUMBER: _ClassVar[int]
    state: ControlManagerState.State
    time_scale: float
    control_state: ControlManagerState.ControlState
    enabled_joint_idx: _containers.RepeatedScalarFieldContainer[int]
    unlimited_mode_enabled: bool
    def __init__(self, state: _Optional[_Union[ControlManagerState.State, str]] = ..., time_scale: _Optional[float] = ..., control_state: _Optional[_Union[ControlManagerState.ControlState, str]] = ..., enabled_joint_idx: _Optional[_Iterable[int]] = ..., unlimited_mode_enabled: bool = ...) -> None: ...

class ControlManagerCommandRequest(_message.Message):
    __slots__ = ("request_header", "command", "unlimited_mode_enabled")
    class Command(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        COMMAND_UNKNOWN: _ClassVar[ControlManagerCommandRequest.Command]
        COMMAND_ENABLE: _ClassVar[ControlManagerCommandRequest.Command]
        COMMAND_DISABLE: _ClassVar[ControlManagerCommandRequest.Command]
        COMMAND_RESET_FAULT: _ClassVar[ControlManagerCommandRequest.Command]
    COMMAND_UNKNOWN: ControlManagerCommandRequest.Command
    COMMAND_ENABLE: ControlManagerCommandRequest.Command
    COMMAND_DISABLE: ControlManagerCommandRequest.Command
    COMMAND_RESET_FAULT: ControlManagerCommandRequest.Command
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    COMMAND_FIELD_NUMBER: _ClassVar[int]
    UNLIMITED_MODE_ENABLED_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    command: ControlManagerCommandRequest.Command
    unlimited_mode_enabled: _wrappers_pb2.BoolValue
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., command: _Optional[_Union[ControlManagerCommandRequest.Command, str]] = ..., unlimited_mode_enabled: _Optional[_Union[_wrappers_pb2.BoolValue, _Mapping]] = ...) -> None: ...

class ControlManagerCommandResponse(_message.Message):
    __slots__ = ("response_header", "control_manager_state")
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    CONTROL_MANAGER_STATE_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    control_manager_state: ControlManagerState
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., control_manager_state: _Optional[_Union[ControlManagerState, _Mapping]] = ...) -> None: ...

class GetTimeScaleRequest(_message.Message):
    __slots__ = ("request_header",)
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ...) -> None: ...

class GetTimeScaleResponse(_message.Message):
    __slots__ = ("response_header", "time_scale")
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    TIME_SCALE_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    time_scale: float
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., time_scale: _Optional[float] = ...) -> None: ...

class SetTimeScaleRequest(_message.Message):
    __slots__ = ("request_header", "time_scale")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    TIME_SCALE_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    time_scale: float
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., time_scale: _Optional[float] = ...) -> None: ...

class SetTimeScaleResponse(_message.Message):
    __slots__ = ("response_header", "current_time_scale")
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    CURRENT_TIME_SCALE_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    current_time_scale: float
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., current_time_scale: _Optional[float] = ...) -> None: ...

class CancelControlRequest(_message.Message):
    __slots__ = ("request_header",)
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ...) -> None: ...

class CancelControlResponse(_message.Message):
    __slots__ = ("response_header",)
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ...) -> None: ...

class WaitForControlReadyRequest(_message.Message):
    __slots__ = ("request_header", "timeout")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    TIMEOUT_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    timeout: _duration_pb2.Duration
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., timeout: _Optional[_Union[_duration_pb2.Duration, _Mapping]] = ...) -> None: ...

class WaitForControlReadyResponse(_message.Message):
    __slots__ = ("response_header", "ready")
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    READY_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    ready: bool
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., ready: bool = ...) -> None: ...
