from rb.api import basic_command_pb2 as _basic_command_pb2
from rb.api import command_header_pb2 as _command_header_pb2
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class WholeBodyCommand(_message.Message):
    __slots__ = ()
    class Request(_message.Message):
        __slots__ = ("command_header", "stop_command", "real_time_control_command")
        COMMAND_HEADER_FIELD_NUMBER: _ClassVar[int]
        STOP_COMMAND_FIELD_NUMBER: _ClassVar[int]
        REAL_TIME_CONTROL_COMMAND_FIELD_NUMBER: _ClassVar[int]
        command_header: _command_header_pb2.CommandHeader.Request
        stop_command: _basic_command_pb2.StopCommand.Request
        real_time_control_command: _basic_command_pb2.RealTimeControlCommand.Request
        def __init__(self, command_header: _Optional[_Union[_command_header_pb2.CommandHeader.Request, _Mapping]] = ..., stop_command: _Optional[_Union[_basic_command_pb2.StopCommand.Request, _Mapping]] = ..., real_time_control_command: _Optional[_Union[_basic_command_pb2.RealTimeControlCommand.Request, _Mapping]] = ...) -> None: ...
    class Feedback(_message.Message):
        __slots__ = ("command_header_feedback", "stop_command_feedback", "real_time_control_command_feedback")
        COMMAND_HEADER_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        STOP_COMMAND_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        REAL_TIME_CONTROL_COMMAND_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        command_header_feedback: _command_header_pb2.CommandHeader.Feedback
        stop_command_feedback: _basic_command_pb2.StopCommand.Feedback
        real_time_control_command_feedback: _basic_command_pb2.RealTimeControlCommand.Feedback
        def __init__(self, command_header_feedback: _Optional[_Union[_command_header_pb2.CommandHeader.Feedback, _Mapping]] = ..., stop_command_feedback: _Optional[_Union[_basic_command_pb2.StopCommand.Feedback, _Mapping]] = ..., real_time_control_command_feedback: _Optional[_Union[_basic_command_pb2.RealTimeControlCommand.Feedback, _Mapping]] = ...) -> None: ...
    def __init__(self) -> None: ...
