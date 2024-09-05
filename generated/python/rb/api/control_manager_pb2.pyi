from rb.api import header_pb2 as _header_pb2
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class ControlManagerState(_message.Message):
    __slots__ = ("state", "time_scale", "control_state")
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
    state: ControlManagerState.State
    time_scale: float
    control_state: ControlManagerState.ControlState
    def __init__(self, state: _Optional[_Union[ControlManagerState.State, str]] = ..., time_scale: _Optional[float] = ..., control_state: _Optional[_Union[ControlManagerState.ControlState, str]] = ...) -> None: ...

class ControlManagerCommandRequest(_message.Message):
    __slots__ = ("request_header", "command")
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
    request_header: _header_pb2.RequestHeader
    command: ControlManagerCommandRequest.Command
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., command: _Optional[_Union[ControlManagerCommandRequest.Command, str]] = ...) -> None: ...

class ControlManagerCommandResponse(_message.Message):
    __slots__ = ("response_header", "control_manager_state")
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    CONTROL_MANAGER_STATE_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    control_manager_state: ControlManagerState
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., control_manager_state: _Optional[_Union[ControlManagerState, _Mapping]] = ...) -> None: ...

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
