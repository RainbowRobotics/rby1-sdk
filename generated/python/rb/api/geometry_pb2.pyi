from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Vec2(_message.Message):
    __slots__ = ("x", "y")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ...) -> None: ...

class Vec3(_message.Message):
    __slots__ = ("x", "y", "z")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    z: float
    def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ..., z: _Optional[float] = ...) -> None: ...

class SE2Pose(_message.Message):
    __slots__ = ("position", "angle")
    POSITION_FIELD_NUMBER: _ClassVar[int]
    ANGLE_FIELD_NUMBER: _ClassVar[int]
    position: Vec2
    angle: float
    def __init__(self, position: _Optional[_Union[Vec2, _Mapping]] = ..., angle: _Optional[float] = ...) -> None: ...

class SE2Velocity(_message.Message):
    __slots__ = ("linear", "angular")
    LINEAR_FIELD_NUMBER: _ClassVar[int]
    ANGULAR_FIELD_NUMBER: _ClassVar[int]
    linear: Vec2
    angular: float
    def __init__(self, linear: _Optional[_Union[Vec2, _Mapping]] = ..., angular: _Optional[float] = ...) -> None: ...

class Quaternion(_message.Message):
    __slots__ = ("x", "y", "z", "w")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    Z_FIELD_NUMBER: _ClassVar[int]
    W_FIELD_NUMBER: _ClassVar[int]
    x: float
    y: float
    z: float
    w: float
    def __init__(self, x: _Optional[float] = ..., y: _Optional[float] = ..., z: _Optional[float] = ..., w: _Optional[float] = ...) -> None: ...

class EulerAngleZYX(_message.Message):
    __slots__ = ("z", "y", "x")
    Z_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    X_FIELD_NUMBER: _ClassVar[int]
    z: float
    y: float
    x: float
    def __init__(self, z: _Optional[float] = ..., y: _Optional[float] = ..., x: _Optional[float] = ...) -> None: ...

class SE3Pose(_message.Message):
    __slots__ = ("position", "quaternion", "euler")
    POSITION_FIELD_NUMBER: _ClassVar[int]
    QUATERNION_FIELD_NUMBER: _ClassVar[int]
    EULER_FIELD_NUMBER: _ClassVar[int]
    position: Vec3
    quaternion: Quaternion
    euler: EulerAngleZYX
    def __init__(self, position: _Optional[_Union[Vec3, _Mapping]] = ..., quaternion: _Optional[_Union[Quaternion, _Mapping]] = ..., euler: _Optional[_Union[EulerAngleZYX, _Mapping]] = ...) -> None: ...
