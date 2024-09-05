from rb.api import command_header_pb2 as _command_header_pb2
from rb.api import basic_command_pb2 as _basic_command_pb2
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class HeadCommand(_message.Message):
    __slots__ = ()
    class Request(_message.Message):
        __slots__ = ("command_header", "joint_position_command")
        COMMAND_HEADER_FIELD_NUMBER: _ClassVar[int]
        JOINT_POSITION_COMMAND_FIELD_NUMBER: _ClassVar[int]
        command_header: _command_header_pb2.CommandHeader.Request
        joint_position_command: _basic_command_pb2.JointPositionCommand.Request
        def __init__(self, command_header: _Optional[_Union[_command_header_pb2.CommandHeader.Request, _Mapping]] = ..., joint_position_command: _Optional[_Union[_basic_command_pb2.JointPositionCommand.Request, _Mapping]] = ...) -> None: ...
    class Feedback(_message.Message):
        __slots__ = ("command_header_feedback", "joint_position_command_feedback")
        COMMAND_HEADER_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        JOINT_POSITION_COMMAND_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        command_header_feedback: _command_header_pb2.CommandHeader.Feedback
        joint_position_command_feedback: _basic_command_pb2.JointPositionCommand.Feedback
        def __init__(self, command_header_feedback: _Optional[_Union[_command_header_pb2.CommandHeader.Feedback, _Mapping]] = ..., joint_position_command_feedback: _Optional[_Union[_basic_command_pb2.JointPositionCommand.Feedback, _Mapping]] = ...) -> None: ...
    def __init__(self) -> None: ...
