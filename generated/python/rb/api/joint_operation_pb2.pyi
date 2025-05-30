from google.protobuf import timestamp_pb2 as _timestamp_pb2
from google.protobuf import wrappers_pb2 as _wrappers_pb2
from rb.api import header_pb2 as _header_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class ServoOnRequest(_message.Message):
    __slots__ = ("request_header", "name")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    name: str
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., name: _Optional[str] = ...) -> None: ...

class ServoOnResponse(_message.Message):
    __slots__ = ("response_header", "status", "message")
    class Status(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        STATUS_UNKNOWN: _ClassVar[ServoOnResponse.Status]
        STATUS_SUCCESS: _ClassVar[ServoOnResponse.Status]
        STATUS_INTERNAL_ERROR: _ClassVar[ServoOnResponse.Status]
    STATUS_UNKNOWN: ServoOnResponse.Status
    STATUS_SUCCESS: ServoOnResponse.Status
    STATUS_INTERNAL_ERROR: ServoOnResponse.Status
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    status: ServoOnResponse.Status
    message: str
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., status: _Optional[_Union[ServoOnResponse.Status, str]] = ..., message: _Optional[str] = ...) -> None: ...

class ServoOffRequest(_message.Message):
    __slots__ = ("request_header", "name")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    name: str
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., name: _Optional[str] = ...) -> None: ...

class ServoOffResponse(_message.Message):
    __slots__ = ("response_header", "status", "message")
    class Status(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        STATUS_UNKNOWN: _ClassVar[ServoOffResponse.Status]
        STATUS_SUCCESS: _ClassVar[ServoOffResponse.Status]
        STATUS_INTERNAL_ERROR: _ClassVar[ServoOffResponse.Status]
    STATUS_UNKNOWN: ServoOffResponse.Status
    STATUS_SUCCESS: ServoOffResponse.Status
    STATUS_INTERNAL_ERROR: ServoOffResponse.Status
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    status: ServoOffResponse.Status
    message: str
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., status: _Optional[_Union[ServoOffResponse.Status, str]] = ..., message: _Optional[str] = ...) -> None: ...

class BrakeEngageRequest(_message.Message):
    __slots__ = ("request_header", "name")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    name: str
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., name: _Optional[str] = ...) -> None: ...

class BrakeEngageResponse(_message.Message):
    __slots__ = ("response_header", "status", "message")
    class Status(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        STATUS_UNKNOWN: _ClassVar[BrakeEngageResponse.Status]
        STATUS_SUCCESS: _ClassVar[BrakeEngageResponse.Status]
        STATUS_INTERNAL_ERROR: _ClassVar[BrakeEngageResponse.Status]
    STATUS_UNKNOWN: BrakeEngageResponse.Status
    STATUS_SUCCESS: BrakeEngageResponse.Status
    STATUS_INTERNAL_ERROR: BrakeEngageResponse.Status
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    status: BrakeEngageResponse.Status
    message: str
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., status: _Optional[_Union[BrakeEngageResponse.Status, str]] = ..., message: _Optional[str] = ...) -> None: ...

class BrakeReleaseRequest(_message.Message):
    __slots__ = ("request_header", "name")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    name: str
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., name: _Optional[str] = ...) -> None: ...

class BrakeReleaseResponse(_message.Message):
    __slots__ = ("response_header", "status", "message")
    class Status(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        STATUS_UNKNOWN: _ClassVar[BrakeReleaseResponse.Status]
        STATUS_SUCCESS: _ClassVar[BrakeReleaseResponse.Status]
        STATUS_INTERNAL_ERROR: _ClassVar[BrakeReleaseResponse.Status]
    STATUS_UNKNOWN: BrakeReleaseResponse.Status
    STATUS_SUCCESS: BrakeReleaseResponse.Status
    STATUS_INTERNAL_ERROR: BrakeReleaseResponse.Status
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    status: BrakeReleaseResponse.Status
    message: str
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., status: _Optional[_Union[BrakeReleaseResponse.Status, str]] = ..., message: _Optional[str] = ...) -> None: ...

class HomeOffsetResetRequest(_message.Message):
    __slots__ = ("request_header", "name")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    name: str
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., name: _Optional[str] = ...) -> None: ...

class HomeOffsetResetResponse(_message.Message):
    __slots__ = ("response_header", "status", "message")
    class Status(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        STATUS_UNKNOWN: _ClassVar[HomeOffsetResetResponse.Status]
        STATUS_SUCCESS: _ClassVar[HomeOffsetResetResponse.Status]
        STATUS_INTERNAL_ERROR: _ClassVar[HomeOffsetResetResponse.Status]
    STATUS_UNKNOWN: HomeOffsetResetResponse.Status
    STATUS_SUCCESS: HomeOffsetResetResponse.Status
    STATUS_INTERNAL_ERROR: HomeOffsetResetResponse.Status
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    status: HomeOffsetResetResponse.Status
    message: str
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., status: _Optional[_Union[HomeOffsetResetResponse.Status, str]] = ..., message: _Optional[str] = ...) -> None: ...

class SetPositionPIDGainRequest(_message.Message):
    __slots__ = ("request_header", "name", "p_gain", "i_gain", "d_gain")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    P_GAIN_FIELD_NUMBER: _ClassVar[int]
    I_GAIN_FIELD_NUMBER: _ClassVar[int]
    D_GAIN_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    name: str
    p_gain: _wrappers_pb2.UInt32Value
    i_gain: _wrappers_pb2.UInt32Value
    d_gain: _wrappers_pb2.UInt32Value
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., name: _Optional[str] = ..., p_gain: _Optional[_Union[_wrappers_pb2.UInt32Value, _Mapping]] = ..., i_gain: _Optional[_Union[_wrappers_pb2.UInt32Value, _Mapping]] = ..., d_gain: _Optional[_Union[_wrappers_pb2.UInt32Value, _Mapping]] = ...) -> None: ...

class SetPositionPIDGainResponse(_message.Message):
    __slots__ = ("response_header", "status", "message")
    class Status(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        STATUS_UNKNOWN: _ClassVar[SetPositionPIDGainResponse.Status]
        STATUS_SUCCESS: _ClassVar[SetPositionPIDGainResponse.Status]
        STATUS_INTERNAL_ERROR: _ClassVar[SetPositionPIDGainResponse.Status]
    STATUS_UNKNOWN: SetPositionPIDGainResponse.Status
    STATUS_SUCCESS: SetPositionPIDGainResponse.Status
    STATUS_INTERNAL_ERROR: SetPositionPIDGainResponse.Status
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    STATUS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    status: SetPositionPIDGainResponse.Status
    message: str
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., status: _Optional[_Union[SetPositionPIDGainResponse.Status, str]] = ..., message: _Optional[str] = ...) -> None: ...

class PositionPIDGain(_message.Message):
    __slots__ = ("timestamp", "p_gain", "i_gain", "d_gain")
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    P_GAIN_FIELD_NUMBER: _ClassVar[int]
    I_GAIN_FIELD_NUMBER: _ClassVar[int]
    D_GAIN_FIELD_NUMBER: _ClassVar[int]
    timestamp: _timestamp_pb2.Timestamp
    p_gain: int
    i_gain: int
    d_gain: int
    def __init__(self, timestamp: _Optional[_Union[_timestamp_pb2.Timestamp, _Mapping]] = ..., p_gain: _Optional[int] = ..., i_gain: _Optional[int] = ..., d_gain: _Optional[int] = ...) -> None: ...

class GetPositionPIDGainRequest(_message.Message):
    __slots__ = ("request_header", "dev_name", "target_component")
    class TargetComponent(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
        __slots__ = ()
        UNKNOWN: _ClassVar[GetPositionPIDGainRequest.TargetComponent]
        TORSO: _ClassVar[GetPositionPIDGainRequest.TargetComponent]
        RIGHT_ARM: _ClassVar[GetPositionPIDGainRequest.TargetComponent]
        LEFT_ARM: _ClassVar[GetPositionPIDGainRequest.TargetComponent]
        HEAD: _ClassVar[GetPositionPIDGainRequest.TargetComponent]
    UNKNOWN: GetPositionPIDGainRequest.TargetComponent
    TORSO: GetPositionPIDGainRequest.TargetComponent
    RIGHT_ARM: GetPositionPIDGainRequest.TargetComponent
    LEFT_ARM: GetPositionPIDGainRequest.TargetComponent
    HEAD: GetPositionPIDGainRequest.TargetComponent
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    DEV_NAME_FIELD_NUMBER: _ClassVar[int]
    TARGET_COMPONENT_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    dev_name: str
    target_component: GetPositionPIDGainRequest.TargetComponent
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., dev_name: _Optional[str] = ..., target_component: _Optional[_Union[GetPositionPIDGainRequest.TargetComponent, str]] = ...) -> None: ...

class GetPositionPIDGainResponse(_message.Message):
    __slots__ = ("response_header", "position_gain")
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    POSITION_GAIN_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    position_gain: _containers.RepeatedCompositeFieldContainer[PositionPIDGain]
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., position_gain: _Optional[_Iterable[_Union[PositionPIDGain, _Mapping]]] = ...) -> None: ...
