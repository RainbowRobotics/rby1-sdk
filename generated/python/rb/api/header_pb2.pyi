from google.protobuf import timestamp_pb2 as _timestamp_pb2
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class RequestHeader(_message.Message):
    __slots__ = ("request_timestamp",)
    REQUEST_TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    request_timestamp: _timestamp_pb2.Timestamp
    def __init__(self, request_timestamp: _Optional[_Union[_timestamp_pb2.Timestamp, _Mapping]] = ...) -> None: ...

class ResponseHeader(_message.Message):
    __slots__ = ("request_header", "request_received_timestamp", "response_timestamp", "error")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    REQUEST_RECEIVED_TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    RESPONSE_TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    ERROR_FIELD_NUMBER: _ClassVar[int]
    request_header: RequestHeader
    request_received_timestamp: _timestamp_pb2.Timestamp
    response_timestamp: _timestamp_pb2.Timestamp
    error: CommonError
    def __init__(self, request_header: _Optional[_Union[RequestHeader, _Mapping]] = ..., request_received_timestamp: _Optional[_Union[_timestamp_pb2.Timestamp, _Mapping]] = ..., response_timestamp: _Optional[_Union[_timestamp_pb2.Timestamp, _Mapping]] = ..., error: _Optional[_Union[CommonError, _Mapping]] = ...) -> None: ...

class CommonError(_message.Message):
    __slots__ = ("code", "message")
    class Code(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        CODE_UNSPECIFIED: _ClassVar[CommonError.Code]
        CODE_OK: _ClassVar[CommonError.Code]
        CODE_INTERNAL_SERVER_ERROR: _ClassVar[CommonError.Code]
        CODE_INVALID_REQUEST: _ClassVar[CommonError.Code]
    CODE_UNSPECIFIED: CommonError.Code
    CODE_OK: CommonError.Code
    CODE_INTERNAL_SERVER_ERROR: CommonError.Code
    CODE_INVALID_REQUEST: CommonError.Code
    CODE_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    code: CommonError.Code
    message: str
    def __init__(self, code: _Optional[_Union[CommonError.Code, str]] = ..., message: _Optional[str] = ...) -> None: ...
