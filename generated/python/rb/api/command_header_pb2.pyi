from google.protobuf import duration_pb2 as _duration_pb2
from rb.api import geometry_pb2 as _geometry_pb2
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class CommandHeader(_message.Message):
    __slots__ = ()
    class Request(_message.Message):
        __slots__ = ("control_hold_time",)
        CONTROL_HOLD_TIME_FIELD_NUMBER: _ClassVar[int]
        control_hold_time: _duration_pb2.Duration
        def __init__(self, control_hold_time: _Optional[_Union[_duration_pb2.Duration, _Mapping]] = ...) -> None: ...
    class Feedback(_message.Message):
        __slots__ = ("finished",)
        FINISHED_FIELD_NUMBER: _ClassVar[int]
        finished: bool
        def __init__(self, finished: bool = ...) -> None: ...
    def __init__(self) -> None: ...
