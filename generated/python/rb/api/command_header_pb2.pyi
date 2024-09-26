from google.protobuf import duration_pb2 as _duration_pb2
from rb.api import geometry_pb2 as _geometry_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class CommandHeader(_message.Message):
    __slots__ = ()
    class Request(_message.Message):
        __slots__ = ("control_hold_time", "gravity", "inertials")
        class InertialsEntry(_message.Message):
            __slots__ = ("key", "value")
            KEY_FIELD_NUMBER: _ClassVar[int]
            VALUE_FIELD_NUMBER: _ClassVar[int]
            key: str
            value: _geometry_pb2.Inertial
            def __init__(self, key: _Optional[str] = ..., value: _Optional[_Union[_geometry_pb2.Inertial, _Mapping]] = ...) -> None: ...
        CONTROL_HOLD_TIME_FIELD_NUMBER: _ClassVar[int]
        GRAVITY_FIELD_NUMBER: _ClassVar[int]
        INERTIALS_FIELD_NUMBER: _ClassVar[int]
        control_hold_time: _duration_pb2.Duration
        gravity: _geometry_pb2.Vec3
        inertials: _containers.MessageMap[str, _geometry_pb2.Inertial]
        def __init__(self, control_hold_time: _Optional[_Union[_duration_pb2.Duration, _Mapping]] = ..., gravity: _Optional[_Union[_geometry_pb2.Vec3, _Mapping]] = ..., inertials: _Optional[_Mapping[str, _geometry_pb2.Inertial]] = ...) -> None: ...
    class Feedback(_message.Message):
        __slots__ = ("finished",)
        FINISHED_FIELD_NUMBER: _ClassVar[int]
        finished: bool
        def __init__(self, finished: bool = ...) -> None: ...
    def __init__(self) -> None: ...
