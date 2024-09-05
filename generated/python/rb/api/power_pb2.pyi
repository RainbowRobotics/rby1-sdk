from rb.api import header_pb2 as _header_pb2
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class PowerCommandRequest(_message.Message):
    __slots__ = ("request_header", "name", "command")
    class Command(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        COMMAND_UNKNOWN: _ClassVar[PowerCommandRequest.Command]
        COMMAND_POWER_ON: _ClassVar[PowerCommandRequest.Command]
        COMMAND_POWER_OFF: _ClassVar[PowerCommandRequest.Command]
    COMMAND_UNKNOWN: PowerCommandRequest.Command
    COMMAND_POWER_ON: PowerCommandRequest.Command
    COMMAND_POWER_OFF: PowerCommandRequest.Command
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    COMMAND_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    name: str
    command: PowerCommandRequest.Command
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., name: _Optional[str] = ..., command: _Optional[_Union[PowerCommandRequest.Command, str]] = ...) -> None: ...

class PowerCommandResponse(_message.Message):
    __slots__ = ("response_header", "status", "message")
    class Status(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        STATUS_UNKNOWN: _ClassVar[PowerCommandResponse.Status]
        STATUS_SUCCESS: _ClassVar[PowerCommandResponse.Status]
        STATUS_INTERNAL_ERROR: _ClassVar[PowerCommandResponse.Status]
    STATUS_UNKNOWN: PowerCommandResponse.Status
    STATUS_SUCCESS: PowerCommandResponse.Status
    STATUS_INTERNAL_ERROR: PowerCommandResponse.Status
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    status: PowerCommandResponse.Status
    message: str
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., status: _Optional[_Union[PowerCommandResponse.Status, str]] = ..., message: _Optional[str] = ...) -> None: ...

class JointCommandRequest(_message.Message):
    __slots__ = ("request_header", "name", "command")
    class Command(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        COMMAND_UNKNOWN: _ClassVar[JointCommandRequest.Command]
        COMMAND_SERVO_ON: _ClassVar[JointCommandRequest.Command]
        COMMAND_BRAKE_ENGAGE: _ClassVar[JointCommandRequest.Command]
        COMMAND_BRAKE_RELEASE: _ClassVar[JointCommandRequest.Command]
        COMMAND_HOME_OFFSET_RST: _ClassVar[JointCommandRequest.Command]
    COMMAND_UNKNOWN: JointCommandRequest.Command
    COMMAND_SERVO_ON: JointCommandRequest.Command
    COMMAND_BRAKE_ENGAGE: JointCommandRequest.Command
    COMMAND_BRAKE_RELEASE: JointCommandRequest.Command
    COMMAND_HOME_OFFSET_RST: JointCommandRequest.Command
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    COMMAND_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    name: str
    command: JointCommandRequest.Command
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., name: _Optional[str] = ..., command: _Optional[_Union[JointCommandRequest.Command, str]] = ...) -> None: ...

class JointCommandResponse(_message.Message):
    __slots__ = ("response_header", "status", "message")
    class Status(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        STATUS_UNKNOWN: _ClassVar[JointCommandResponse.Status]
        STATUS_SUCCESS: _ClassVar[JointCommandResponse.Status]
        STATUS_INTERNAL_ERROR: _ClassVar[JointCommandResponse.Status]
    STATUS_UNKNOWN: JointCommandResponse.Status
    STATUS_SUCCESS: JointCommandResponse.Status
    STATUS_INTERNAL_ERROR: JointCommandResponse.Status
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    status: JointCommandResponse.Status
    message: str
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., status: _Optional[_Union[JointCommandResponse.Status, str]] = ..., message: _Optional[str] = ...) -> None: ...

class ToolFlangePowerCommandRequest(_message.Message):
    __slots__ = ("request_header", "name", "command")
    class Command(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        COMMAND_UNKNOWN: _ClassVar[ToolFlangePowerCommandRequest.Command]
        COMMAND_POWER_OFF: _ClassVar[ToolFlangePowerCommandRequest.Command]
        COMMAND_POWER_12V: _ClassVar[ToolFlangePowerCommandRequest.Command]
        COMMAND_POWER_24V: _ClassVar[ToolFlangePowerCommandRequest.Command]
    COMMAND_UNKNOWN: ToolFlangePowerCommandRequest.Command
    COMMAND_POWER_OFF: ToolFlangePowerCommandRequest.Command
    COMMAND_POWER_12V: ToolFlangePowerCommandRequest.Command
    COMMAND_POWER_24V: ToolFlangePowerCommandRequest.Command
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    COMMAND_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    name: str
    command: ToolFlangePowerCommandRequest.Command
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., name: _Optional[str] = ..., command: _Optional[_Union[ToolFlangePowerCommandRequest.Command, str]] = ...) -> None: ...

class ToolFlangePowerCommandResponse(_message.Message):
    __slots__ = ("response_header",)
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ...) -> None: ...
