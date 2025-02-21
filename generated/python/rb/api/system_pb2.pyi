from google.protobuf import timestamp_pb2 as _timestamp_pb2
from rb.api import header_pb2 as _header_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class GetSystemTimeRequest(_message.Message):
    __slots__ = ("request_header",)
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ...) -> None: ...

class GetSystemTimeResponse(_message.Message):
    __slots__ = ("response_header", "utc_time", "time_zone", "local_time")
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    UTC_TIME_FIELD_NUMBER: _ClassVar[int]
    TIME_ZONE_FIELD_NUMBER: _ClassVar[int]
    LOCAL_TIME_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    utc_time: _timestamp_pb2.Timestamp
    time_zone: str
    local_time: str
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., utc_time: _Optional[_Union[_timestamp_pb2.Timestamp, _Mapping]] = ..., time_zone: _Optional[str] = ..., local_time: _Optional[str] = ...) -> None: ...

class SetSystemTimeRequest(_message.Message):
    __slots__ = ("request_header", "utc_time", "time_zone")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    UTC_TIME_FIELD_NUMBER: _ClassVar[int]
    TIME_ZONE_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    utc_time: _timestamp_pb2.Timestamp
    time_zone: str
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., utc_time: _Optional[_Union[_timestamp_pb2.Timestamp, _Mapping]] = ..., time_zone: _Optional[str] = ...) -> None: ...

class SetSystemTimeResponse(_message.Message):
    __slots__ = ("response_header",)
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ...) -> None: ...

class SetBatteryLevelRequest(_message.Message):
    __slots__ = ("request_header", "level")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    LEVEL_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    level: float
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., level: _Optional[float] = ...) -> None: ...

class SetBatteryLevelResponse(_message.Message):
    __slots__ = ("response_header",)
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ...) -> None: ...

class SetBatteryConfigRequest(_message.Message):
    __slots__ = ("request_header", "cut_off_voltage", "fully_charged_voltage", "coefficients")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    CUT_OFF_VOLTAGE_FIELD_NUMBER: _ClassVar[int]
    FULLY_CHARGED_VOLTAGE_FIELD_NUMBER: _ClassVar[int]
    COEFFICIENTS_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    cut_off_voltage: float
    fully_charged_voltage: float
    coefficients: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., cut_off_voltage: _Optional[float] = ..., fully_charged_voltage: _Optional[float] = ..., coefficients: _Optional[_Iterable[float]] = ...) -> None: ...

class SetBatteryConfigResponse(_message.Message):
    __slots__ = ("response_header",)
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ...) -> None: ...

class ResetBatteryConfigRequest(_message.Message):
    __slots__ = ("request_header",)
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ...) -> None: ...

class ResetBatteryConfigResponse(_message.Message):
    __slots__ = ("response_header",)
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ...) -> None: ...
