from rb.api import color_pb2 as _color_pb2
from rb.api import header_pb2 as _header_pb2
from google.protobuf import duration_pb2 as _duration_pb2
from google.protobuf import wrappers_pb2 as _wrappers_pb2
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class SetLEDColorRequest(_message.Message):
    __slots__ = ("request_header", "color", "duration", "transition_time", "blinking", "blinking_freq")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    COLOR_FIELD_NUMBER: _ClassVar[int]
    DURATION_FIELD_NUMBER: _ClassVar[int]
    TRANSITION_TIME_FIELD_NUMBER: _ClassVar[int]
    BLINKING_FIELD_NUMBER: _ClassVar[int]
    BLINKING_FREQ_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    color: _color_pb2.Color
    duration: _duration_pb2.Duration
    transition_time: _duration_pb2.Duration
    blinking: bool
    blinking_freq: _wrappers_pb2.DoubleValue
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., color: _Optional[_Union[_color_pb2.Color, _Mapping]] = ..., duration: _Optional[_Union[_duration_pb2.Duration, _Mapping]] = ..., transition_time: _Optional[_Union[_duration_pb2.Duration, _Mapping]] = ..., blinking: bool = ..., blinking_freq: _Optional[_Union[_wrappers_pb2.DoubleValue, _Mapping]] = ...) -> None: ...

class SetLEDColorResponse(_message.Message):
    __slots__ = ("response_header",)
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ...) -> None: ...
