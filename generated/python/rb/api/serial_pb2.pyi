from rb.api import header_pb2 as _header_pb2
from google.protobuf import wrappers_pb2 as _wrappers_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class SerialDeviceInfo(_message.Message):
    __slots__ = ("path", "description")
    PATH_FIELD_NUMBER: _ClassVar[int]
    DESCRIPTION_FIELD_NUMBER: _ClassVar[int]
    path: str
    description: str
    def __init__(self, path: _Optional[str] = ..., description: _Optional[str] = ...) -> None: ...

class SerialOpenRequest(_message.Message):
    __slots__ = ("device_path", "baudrate", "bytesize", "parity", "stopbits")
    DEVICE_PATH_FIELD_NUMBER: _ClassVar[int]
    BAUDRATE_FIELD_NUMBER: _ClassVar[int]
    BYTESIZE_FIELD_NUMBER: _ClassVar[int]
    PARITY_FIELD_NUMBER: _ClassVar[int]
    STOPBITS_FIELD_NUMBER: _ClassVar[int]
    device_path: str
    baudrate: int
    bytesize: _wrappers_pb2.Int32Value
    parity: _wrappers_pb2.Int32Value
    stopbits: _wrappers_pb2.Int32Value
    def __init__(self, device_path: _Optional[str] = ..., baudrate: _Optional[int] = ..., bytesize: _Optional[_Union[_wrappers_pb2.Int32Value, _Mapping]] = ..., parity: _Optional[_Union[_wrappers_pb2.Int32Value, _Mapping]] = ..., stopbits: _Optional[_Union[_wrappers_pb2.Int32Value, _Mapping]] = ...) -> None: ...

class SerialWriteRequest(_message.Message):
    __slots__ = ("data",)
    DATA_FIELD_NUMBER: _ClassVar[int]
    data: bytes
    def __init__(self, data: _Optional[bytes] = ...) -> None: ...

class SerialConnectionStatus(_message.Message):
    __slots__ = ("success", "message")
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    success: bool
    message: str
    def __init__(self, success: bool = ..., message: _Optional[str] = ...) -> None: ...

class SerialWriteResult(_message.Message):
    __slots__ = ("success", "message")
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    success: bool
    message: str
    def __init__(self, success: bool = ..., message: _Optional[str] = ...) -> None: ...

class GetSerialDeviceListRequest(_message.Message):
    __slots__ = ("request_header",)
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ...) -> None: ...

class GetSerialDeviceListResponse(_message.Message):
    __slots__ = ("response_header", "devices")
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    DEVICES_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    devices: _containers.RepeatedCompositeFieldContainer[SerialDeviceInfo]
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., devices: _Optional[_Iterable[_Union[SerialDeviceInfo, _Mapping]]] = ...) -> None: ...

class OpenSerialStreamRequest(_message.Message):
    __slots__ = ("request_header", "connect", "write")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    CONNECT_FIELD_NUMBER: _ClassVar[int]
    WRITE_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    connect: SerialOpenRequest
    write: SerialWriteRequest
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., connect: _Optional[_Union[SerialOpenRequest, _Mapping]] = ..., write: _Optional[_Union[SerialWriteRequest, _Mapping]] = ...) -> None: ...

class OpenSerialStreamResponse(_message.Message):
    __slots__ = ("response_header", "connect_result", "write_result", "read_data")
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    CONNECT_RESULT_FIELD_NUMBER: _ClassVar[int]
    WRITE_RESULT_FIELD_NUMBER: _ClassVar[int]
    READ_DATA_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    connect_result: SerialConnectionStatus
    write_result: SerialWriteResult
    read_data: bytes
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., connect_result: _Optional[_Union[SerialConnectionStatus, _Mapping]] = ..., write_result: _Optional[_Union[SerialWriteResult, _Mapping]] = ..., read_data: _Optional[bytes] = ...) -> None: ...
