from rb.api import header_pb2 as _header_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Gamepad(_message.Message):
    __slots__ = ("buttons", "joystick")
    BUTTONS_FIELD_NUMBER: _ClassVar[int]
    JOYSTICK_FIELD_NUMBER: _ClassVar[int]
    buttons: _containers.RepeatedScalarFieldContainer[bool]
    joystick: _containers.RepeatedScalarFieldContainer[float]
    def __init__(self, buttons: _Optional[_Iterable[bool]] = ..., joystick: _Optional[_Iterable[float]] = ...) -> None: ...

class UploadGamepadDataRequest(_message.Message):
    __slots__ = ("request_header", "data")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    DATA_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    data: Gamepad
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., data: _Optional[_Union[Gamepad, _Mapping]] = ...) -> None: ...

class UploadGamepadDataResponse(_message.Message):
    __slots__ = ("response_header",)
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ...) -> None: ...
