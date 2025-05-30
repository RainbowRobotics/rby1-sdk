from rb.api import header_pb2 as _header_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class BatteryInfo(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class PowerInfo(_message.Message):
    __slots__ = ("name",)
    NAME_FIELD_NUMBER: _ClassVar[int]
    name: str
    def __init__(self, name: _Optional[str] = ...) -> None: ...

class EMOInfo(_message.Message):
    __slots__ = ("name",)
    NAME_FIELD_NUMBER: _ClassVar[int]
    name: str
    def __init__(self, name: _Optional[str] = ...) -> None: ...

class JointInfo(_message.Message):
    __slots__ = ("name", "has_brake", "product_name", "firmware_version")
    NAME_FIELD_NUMBER: _ClassVar[int]
    HAS_BRAKE_FIELD_NUMBER: _ClassVar[int]
    PRODUCT_NAME_FIELD_NUMBER: _ClassVar[int]
    FIRMWARE_VERSION_FIELD_NUMBER: _ClassVar[int]
    name: str
    has_brake: bool
    product_name: str
    firmware_version: str
    def __init__(self, name: _Optional[str] = ..., has_brake: bool = ..., product_name: _Optional[str] = ..., firmware_version: _Optional[str] = ...) -> None: ...

class RobotInfo(_message.Message):
    __slots__ = ("robot_version", "robot_model_name", "sdk_commit_id", "battery_info", "power_infos", "emo_infos", "degree_of_freedom", "joint_infos", "mobility_joint_idx", "body_joint_idx", "head_joint_idx", "torso_joint_idx", "right_arm_joint_idx", "left_arm_joint_idx")
    ROBOT_VERSION_FIELD_NUMBER: _ClassVar[int]
    ROBOT_MODEL_NAME_FIELD_NUMBER: _ClassVar[int]
    SDK_COMMIT_ID_FIELD_NUMBER: _ClassVar[int]
    BATTERY_INFO_FIELD_NUMBER: _ClassVar[int]
    POWER_INFOS_FIELD_NUMBER: _ClassVar[int]
    EMO_INFOS_FIELD_NUMBER: _ClassVar[int]
    DEGREE_OF_FREEDOM_FIELD_NUMBER: _ClassVar[int]
    JOINT_INFOS_FIELD_NUMBER: _ClassVar[int]
    MOBILITY_JOINT_IDX_FIELD_NUMBER: _ClassVar[int]
    BODY_JOINT_IDX_FIELD_NUMBER: _ClassVar[int]
    HEAD_JOINT_IDX_FIELD_NUMBER: _ClassVar[int]
    TORSO_JOINT_IDX_FIELD_NUMBER: _ClassVar[int]
    RIGHT_ARM_JOINT_IDX_FIELD_NUMBER: _ClassVar[int]
    LEFT_ARM_JOINT_IDX_FIELD_NUMBER: _ClassVar[int]
    robot_version: str
    robot_model_name: str
    sdk_commit_id: str
    battery_info: BatteryInfo
    power_infos: _containers.RepeatedCompositeFieldContainer[PowerInfo]
    emo_infos: _containers.RepeatedCompositeFieldContainer[EMOInfo]
    degree_of_freedom: int
    joint_infos: _containers.RepeatedCompositeFieldContainer[JointInfo]
    mobility_joint_idx: _containers.RepeatedScalarFieldContainer[int]
    body_joint_idx: _containers.RepeatedScalarFieldContainer[int]
    head_joint_idx: _containers.RepeatedScalarFieldContainer[int]
    torso_joint_idx: _containers.RepeatedScalarFieldContainer[int]
    right_arm_joint_idx: _containers.RepeatedScalarFieldContainer[int]
    left_arm_joint_idx: _containers.RepeatedScalarFieldContainer[int]
    def __init__(self, robot_version: _Optional[str] = ..., robot_model_name: _Optional[str] = ..., sdk_commit_id: _Optional[str] = ..., battery_info: _Optional[_Union[BatteryInfo, _Mapping]] = ..., power_infos: _Optional[_Iterable[_Union[PowerInfo, _Mapping]]] = ..., emo_infos: _Optional[_Iterable[_Union[EMOInfo, _Mapping]]] = ..., degree_of_freedom: _Optional[int] = ..., joint_infos: _Optional[_Iterable[_Union[JointInfo, _Mapping]]] = ..., mobility_joint_idx: _Optional[_Iterable[int]] = ..., body_joint_idx: _Optional[_Iterable[int]] = ..., head_joint_idx: _Optional[_Iterable[int]] = ..., torso_joint_idx: _Optional[_Iterable[int]] = ..., right_arm_joint_idx: _Optional[_Iterable[int]] = ..., left_arm_joint_idx: _Optional[_Iterable[int]] = ...) -> None: ...

class GetRobotInfoRequest(_message.Message):
    __slots__ = ("request_header",)
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ...) -> None: ...

class GetRobotInfoResponse(_message.Message):
    __slots__ = ("response_header", "robot_info")
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    ROBOT_INFO_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    robot_info: RobotInfo
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., robot_info: _Optional[_Union[RobotInfo, _Mapping]] = ...) -> None: ...

class GetRobotModelRequest(_message.Message):
    __slots__ = ("request_header",)
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ...) -> None: ...

class GetRobotModelResponse(_message.Message):
    __slots__ = ("response_header", "model")
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    MODEL_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    model: str
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., model: _Optional[str] = ...) -> None: ...

class ImportRobotModelRequest(_message.Message):
    __slots__ = ("request_header", "name", "model")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    NAME_FIELD_NUMBER: _ClassVar[int]
    MODEL_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    name: str
    model: str
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., name: _Optional[str] = ..., model: _Optional[str] = ...) -> None: ...

class ImportRobotModelResponse(_message.Message):
    __slots__ = ("response_header",)
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ...) -> None: ...
