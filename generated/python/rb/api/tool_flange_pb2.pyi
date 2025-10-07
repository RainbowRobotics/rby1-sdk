from rb.api import header_pb2 as _header_pb2
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class SetToolFlangeDigitalOutputRequest(_message.Message):
    __slots__ = ("request_header", "name", "single", "dual")
    class SingleChannel(_message.Message):
        __slots__ = ("channel", "state")
        CHANNEL_FIELD_NUMBER: _ClassVar[int]
        STATE_FIELD_NUMBER: _ClassVar[int]
        channel: int
        state: bool
        def __init__(self, channel: _Optional[int] = ..., state: bool = ...) -> None: ...
    class DualChannel(_message.Message):
        __slots__ = ("state_0", "state_1")
        STATE_0_FIELD_NUMBER: _ClassVar[int]
        STATE_1_FIELD_NUMBER: _ClassVar[int]
        state_0: bool
        state_1: bool
        def __init__(self, state_0: bool = ..., state_1: bool = ...) -> None: ...
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    SINGLE_FIELD_NUMBER: _ClassVar[int]
    DUAL_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    name: str
    single: SetToolFlangeDigitalOutputRequest.SingleChannel
    dual: SetToolFlangeDigitalOutputRequest.DualChannel
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., name: _Optional[str] = ..., single: _Optional[_Union[SetToolFlangeDigitalOutputRequest.SingleChannel, _Mapping]] = ..., dual: _Optional[_Union[SetToolFlangeDigitalOutputRequest.DualChannel, _Mapping]] = ...) -> None: ...

class SetToolFlangeDigitalOutputResponse(_message.Message):
    __slots__ = ("response_header",)
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ...) -> None: ...
