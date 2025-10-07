from rb.api import header_pb2 as _header_pb2
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class DownloadFileRequest(_message.Message):
    __slots__ = ("request_header", "file_path")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    FILE_PATH_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    file_path: str
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., file_path: _Optional[str] = ...) -> None: ...

class DownloadFileResponse(_message.Message):
    __slots__ = ("response_header", "file_content")
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    FILE_CONTENT_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    file_content: bytes
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., file_content: _Optional[bytes] = ...) -> None: ...
