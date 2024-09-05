from rb.api import command_header_pb2 as _command_header_pb2
from rb.api import arm_command_pb2 as _arm_command_pb2
from rb.api import torso_command_pb2 as _torso_command_pb2
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class BodyComponentBasedCommand(_message.Message):
    __slots__ = ()
    class Request(_message.Message):
        __slots__ = ("command_header", "right_arm_command", "left_arm_command", "torso_command")
        COMMAND_HEADER_FIELD_NUMBER: _ClassVar[int]
        RIGHT_ARM_COMMAND_FIELD_NUMBER: _ClassVar[int]
        LEFT_ARM_COMMAND_FIELD_NUMBER: _ClassVar[int]
        TORSO_COMMAND_FIELD_NUMBER: _ClassVar[int]
        command_header: _command_header_pb2.CommandHeader.Request
        right_arm_command: _arm_command_pb2.ArmCommand.Request
        left_arm_command: _arm_command_pb2.ArmCommand.Request
        torso_command: _torso_command_pb2.TorsoCommand.Request
        def __init__(self, command_header: _Optional[_Union[_command_header_pb2.CommandHeader.Request, _Mapping]] = ..., right_arm_command: _Optional[_Union[_arm_command_pb2.ArmCommand.Request, _Mapping]] = ..., left_arm_command: _Optional[_Union[_arm_command_pb2.ArmCommand.Request, _Mapping]] = ..., torso_command: _Optional[_Union[_torso_command_pb2.TorsoCommand.Request, _Mapping]] = ...) -> None: ...
    class Feedback(_message.Message):
        __slots__ = ("command_header_feedback", "right_arm_command_feedback", "left_arm_command_feedback", "torso_command_feedback")
        COMMAND_HEADER_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        RIGHT_ARM_COMMAND_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        LEFT_ARM_COMMAND_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        TORSO_COMMAND_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        command_header_feedback: _command_header_pb2.CommandHeader.Feedback
        right_arm_command_feedback: _arm_command_pb2.ArmCommand.Feedback
        left_arm_command_feedback: _arm_command_pb2.ArmCommand.Feedback
        torso_command_feedback: _torso_command_pb2.TorsoCommand.Feedback
        def __init__(self, command_header_feedback: _Optional[_Union[_command_header_pb2.CommandHeader.Feedback, _Mapping]] = ..., right_arm_command_feedback: _Optional[_Union[_arm_command_pb2.ArmCommand.Feedback, _Mapping]] = ..., left_arm_command_feedback: _Optional[_Union[_arm_command_pb2.ArmCommand.Feedback, _Mapping]] = ..., torso_command_feedback: _Optional[_Union[_torso_command_pb2.TorsoCommand.Feedback, _Mapping]] = ...) -> None: ...
    def __init__(self) -> None: ...
