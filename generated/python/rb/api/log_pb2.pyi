from google.protobuf import timestamp_pb2 as _timestamp_pb2
from rb.api import header_pb2 as _header_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Log(_message.Message):
    __slots__ = ("timestamp", "robot_system_timestamp", "level", "message")
    class Level(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        LEVEL_TRACE: _ClassVar[Log.Level]
        LEVEL_DEBUG: _ClassVar[Log.Level]
        LEVEL_INFO: _ClassVar[Log.Level]
        LEVEL_WARN: _ClassVar[Log.Level]
        LEVEL_ERROR: _ClassVar[Log.Level]
        LEVEL_CRITICAL: _ClassVar[Log.Level]
    LEVEL_TRACE: Log.Level
    LEVEL_DEBUG: Log.Level
    LEVEL_INFO: Log.Level
    LEVEL_WARN: Log.Level
    LEVEL_ERROR: Log.Level
    LEVEL_CRITICAL: Log.Level
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    ROBOT_SYSTEM_TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    LEVEL_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    timestamp: _timestamp_pb2.Timestamp
    robot_system_timestamp: _timestamp_pb2.Timestamp
    level: Log.Level
    message: str
    def __init__(self, timestamp: _Optional[_Union[_timestamp_pb2.Timestamp, _Mapping]] = ..., robot_system_timestamp: _Optional[_Union[_timestamp_pb2.Timestamp, _Mapping]] = ..., level: _Optional[_Union[Log.Level, str]] = ..., message: _Optional[str] = ...) -> None: ...

class GetLastLogRequest(_message.Message):
    __slots__ = ("request_header", "log_count")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    LOG_COUNT_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    log_count: int
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., log_count: _Optional[int] = ...) -> None: ...

class GetLastLogResponse(_message.Message):
    __slots__ = ("response_header", "logs")
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    LOGS_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    logs: _containers.RepeatedCompositeFieldContainer[Log]
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., logs: _Optional[_Iterable[_Union[Log, _Mapping]]] = ...) -> None: ...

class GetLogStreamRequest(_message.Message):
    __slots__ = ("request_header", "update_rate")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    UPDATE_RATE_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    update_rate: float
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., update_rate: _Optional[float] = ...) -> None: ...

class GetLogStreamResponse(_message.Message):
    __slots__ = ("response_header", "logs")
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    LOGS_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    logs: _containers.RepeatedCompositeFieldContainer[Log]
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., logs: _Optional[_Iterable[_Union[Log, _Mapping]]] = ...) -> None: ...

class SetLogLevelRequest(_message.Message):
    __slots__ = ("request_header", "level")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    LEVEL_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    level: Log.Level
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., level: _Optional[_Union[Log.Level, str]] = ...) -> None: ...

class SetLogLevelResponse(_message.Message):
    __slots__ = ("response_header",)
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ...) -> None: ...
