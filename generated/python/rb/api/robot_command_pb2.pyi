from rb.api import header_pb2 as _header_pb2
from rb.api import basic_command_pb2 as _basic_command_pb2
from rb.api import command_header_pb2 as _command_header_pb2
from rb.api import whole_body_command_pb2 as _whole_body_command_pb2
from rb.api import component_based_command_pb2 as _component_based_command_pb2
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class RobotCommand(_message.Message):
    __slots__ = ()
    class Request(_message.Message):
        __slots__ = ("command_header", "whole_body_command", "component_based_command", "jog_command")
        COMMAND_HEADER_FIELD_NUMBER: _ClassVar[int]
        WHOLE_BODY_COMMAND_FIELD_NUMBER: _ClassVar[int]
        COMPONENT_BASED_COMMAND_FIELD_NUMBER: _ClassVar[int]
        JOG_COMMAND_FIELD_NUMBER: _ClassVar[int]
        command_header: _command_header_pb2.CommandHeader.Request
        whole_body_command: _whole_body_command_pb2.WholeBodyCommand.Request
        component_based_command: _component_based_command_pb2.ComponentBasedCommand.Request
        jog_command: _basic_command_pb2.JogCommand.Request
        def __init__(self, command_header: _Optional[_Union[_command_header_pb2.CommandHeader.Request, _Mapping]] = ..., whole_body_command: _Optional[_Union[_whole_body_command_pb2.WholeBodyCommand.Request, _Mapping]] = ..., component_based_command: _Optional[_Union[_component_based_command_pb2.ComponentBasedCommand.Request, _Mapping]] = ..., jog_command: _Optional[_Union[_basic_command_pb2.JogCommand.Request, _Mapping]] = ...) -> None: ...
    class Feedback(_message.Message):
        __slots__ = ("command_header_feedback", "whole_body_command_feedback", "component_based_command_feedback", "jog_command_feedback", "status", "finish_code")
        class Status(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
            __slots__ = ()
            STATUS_IDLE: _ClassVar[RobotCommand.Feedback.Status]
            STATUS_INITIALIZING: _ClassVar[RobotCommand.Feedback.Status]
            STATUS_RUNNING: _ClassVar[RobotCommand.Feedback.Status]
            STATUS_FINISHED: _ClassVar[RobotCommand.Feedback.Status]
        STATUS_IDLE: RobotCommand.Feedback.Status
        STATUS_INITIALIZING: RobotCommand.Feedback.Status
        STATUS_RUNNING: RobotCommand.Feedback.Status
        STATUS_FINISHED: RobotCommand.Feedback.Status
        class FinishCode(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
            __slots__ = ()
            FINISH_CODE_UNKNOWN: _ClassVar[RobotCommand.Feedback.FinishCode]
            FINISH_CODE_OK: _ClassVar[RobotCommand.Feedback.FinishCode]
            FINISH_CODE_CANCELED: _ClassVar[RobotCommand.Feedback.FinishCode]
            FINISH_CODE_PREEMPTED: _ClassVar[RobotCommand.Feedback.FinishCode]
            FINISH_CODE_INITIALIZED_FAILED: _ClassVar[RobotCommand.Feedback.FinishCode]
            FINISH_CODE_CONTROL_MANAGER_IDLE: _ClassVar[RobotCommand.Feedback.FinishCode]
            FINISH_CODE_CONTROL_MANAGER_FAULT: _ClassVar[RobotCommand.Feedback.FinishCode]
            FINISH_CODE_UNEXPECTED_STATE: _ClassVar[RobotCommand.Feedback.FinishCode]
        FINISH_CODE_UNKNOWN: RobotCommand.Feedback.FinishCode
        FINISH_CODE_OK: RobotCommand.Feedback.FinishCode
        FINISH_CODE_CANCELED: RobotCommand.Feedback.FinishCode
        FINISH_CODE_PREEMPTED: RobotCommand.Feedback.FinishCode
        FINISH_CODE_INITIALIZED_FAILED: RobotCommand.Feedback.FinishCode
        FINISH_CODE_CONTROL_MANAGER_IDLE: RobotCommand.Feedback.FinishCode
        FINISH_CODE_CONTROL_MANAGER_FAULT: RobotCommand.Feedback.FinishCode
        FINISH_CODE_UNEXPECTED_STATE: RobotCommand.Feedback.FinishCode
        COMMAND_HEADER_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        WHOLE_BODY_COMMAND_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        COMPONENT_BASED_COMMAND_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        JOG_COMMAND_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        STATUS_FIELD_NUMBER: _ClassVar[int]
        FINISH_CODE_FIELD_NUMBER: _ClassVar[int]
        command_header_feedback: _command_header_pb2.CommandHeader.Feedback
        whole_body_command_feedback: _whole_body_command_pb2.WholeBodyCommand.Feedback
        component_based_command_feedback: _component_based_command_pb2.ComponentBasedCommand.Feedback
        jog_command_feedback: _basic_command_pb2.JogCommand.Feedback
        status: RobotCommand.Feedback.Status
        finish_code: RobotCommand.Feedback.FinishCode
        def __init__(self, command_header_feedback: _Optional[_Union[_command_header_pb2.CommandHeader.Feedback, _Mapping]] = ..., whole_body_command_feedback: _Optional[_Union[_whole_body_command_pb2.WholeBodyCommand.Feedback, _Mapping]] = ..., component_based_command_feedback: _Optional[_Union[_component_based_command_pb2.ComponentBasedCommand.Feedback, _Mapping]] = ..., jog_command_feedback: _Optional[_Union[_basic_command_pb2.JogCommand.Feedback, _Mapping]] = ..., status: _Optional[_Union[RobotCommand.Feedback.Status, str]] = ..., finish_code: _Optional[_Union[RobotCommand.Feedback.FinishCode, str]] = ...) -> None: ...
    def __init__(self) -> None: ...

class RobotCommandRequest(_message.Message):
    __slots__ = ("request_header", "robot_command", "priority")
    REQUEST_HEADER_FIELD_NUMBER: _ClassVar[int]
    ROBOT_COMMAND_FIELD_NUMBER: _ClassVar[int]
    PRIORITY_FIELD_NUMBER: _ClassVar[int]
    request_header: _header_pb2.RequestHeader
    robot_command: RobotCommand.Request
    priority: int
    def __init__(self, request_header: _Optional[_Union[_header_pb2.RequestHeader, _Mapping]] = ..., robot_command: _Optional[_Union[RobotCommand.Request, _Mapping]] = ..., priority: _Optional[int] = ...) -> None: ...

class RobotCommandResponse(_message.Message):
    __slots__ = ("response_header", "feedback")
    RESPONSE_HEADER_FIELD_NUMBER: _ClassVar[int]
    FEEDBACK_FIELD_NUMBER: _ClassVar[int]
    response_header: _header_pb2.ResponseHeader
    feedback: RobotCommand.Feedback
    def __init__(self, response_header: _Optional[_Union[_header_pb2.ResponseHeader, _Mapping]] = ..., feedback: _Optional[_Union[RobotCommand.Feedback, _Mapping]] = ...) -> None: ...
