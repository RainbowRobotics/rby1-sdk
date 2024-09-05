from rb.api import mobility_command_pb2 as _mobility_command_pb2
from rb.api import body_command_pb2 as _body_command_pb2
from rb.api import head_command_pb2 as _head_command_pb2
from rb.api import command_header_pb2 as _command_header_pb2
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class ComponentBasedCommand(_message.Message):
    __slots__ = ()
    class Request(_message.Message):
        __slots__ = ("command_header", "mobility_command", "body_command", "head_command")
        COMMAND_HEADER_FIELD_NUMBER: _ClassVar[int]
        MOBILITY_COMMAND_FIELD_NUMBER: _ClassVar[int]
        BODY_COMMAND_FIELD_NUMBER: _ClassVar[int]
        HEAD_COMMAND_FIELD_NUMBER: _ClassVar[int]
        command_header: _command_header_pb2.CommandHeader.Request
        mobility_command: _mobility_command_pb2.MobilityCommand.Request
        body_command: _body_command_pb2.BodyCommand.Request
        head_command: _head_command_pb2.HeadCommand.Request
        def __init__(self, command_header: _Optional[_Union[_command_header_pb2.CommandHeader.Request, _Mapping]] = ..., mobility_command: _Optional[_Union[_mobility_command_pb2.MobilityCommand.Request, _Mapping]] = ..., body_command: _Optional[_Union[_body_command_pb2.BodyCommand.Request, _Mapping]] = ..., head_command: _Optional[_Union[_head_command_pb2.HeadCommand.Request, _Mapping]] = ...) -> None: ...
    class Feedback(_message.Message):
        __slots__ = ("command_header_feedback", "mobility_command_feedback", "body_command_feedback", "head_command_feedback")
        COMMAND_HEADER_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        MOBILITY_COMMAND_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        BODY_COMMAND_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        HEAD_COMMAND_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        command_header_feedback: _command_header_pb2.CommandHeader.Feedback
        mobility_command_feedback: _mobility_command_pb2.MobilityCommand.Feedback
        body_command_feedback: _body_command_pb2.BodyCommand.Feedback
        head_command_feedback: _head_command_pb2.HeadCommand.Feedback
        def __init__(self, command_header_feedback: _Optional[_Union[_command_header_pb2.CommandHeader.Feedback, _Mapping]] = ..., mobility_command_feedback: _Optional[_Union[_mobility_command_pb2.MobilityCommand.Feedback, _Mapping]] = ..., body_command_feedback: _Optional[_Union[_body_command_pb2.BodyCommand.Feedback, _Mapping]] = ..., head_command_feedback: _Optional[_Union[_head_command_pb2.HeadCommand.Feedback, _Mapping]] = ...) -> None: ...
    def __init__(self) -> None: ...
