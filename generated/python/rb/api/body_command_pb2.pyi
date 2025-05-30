from rb.api import basic_command_pb2 as _basic_command_pb2
from rb.api import command_header_pb2 as _command_header_pb2
from rb.api import body_component_based_command_pb2 as _body_component_based_command_pb2
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class BodyCommand(_message.Message):
    __slots__ = ()
    class Request(_message.Message):
        __slots__ = ("command_header", "joint_position_command", "optimal_control_command", "gravity_compensation_command", "cartesian_command", "body_component_based_command", "joint_impedance_control_command", "cartesian_impedance_control_command")
        COMMAND_HEADER_FIELD_NUMBER: _ClassVar[int]
        JOINT_POSITION_COMMAND_FIELD_NUMBER: _ClassVar[int]
        OPTIMAL_CONTROL_COMMAND_FIELD_NUMBER: _ClassVar[int]
        GRAVITY_COMPENSATION_COMMAND_FIELD_NUMBER: _ClassVar[int]
        CARTESIAN_COMMAND_FIELD_NUMBER: _ClassVar[int]
        BODY_COMPONENT_BASED_COMMAND_FIELD_NUMBER: _ClassVar[int]
        JOINT_IMPEDANCE_CONTROL_COMMAND_FIELD_NUMBER: _ClassVar[int]
        CARTESIAN_IMPEDANCE_CONTROL_COMMAND_FIELD_NUMBER: _ClassVar[int]
        command_header: _command_header_pb2.CommandHeader.Request
        joint_position_command: _basic_command_pb2.JointPositionCommand.Request
        optimal_control_command: _basic_command_pb2.OptimalControlCommand.Request
        gravity_compensation_command: _basic_command_pb2.GravityCompensationCommand.Request
        cartesian_command: _basic_command_pb2.CartesianCommand.Request
        body_component_based_command: _body_component_based_command_pb2.BodyComponentBasedCommand.Request
        joint_impedance_control_command: _basic_command_pb2.JointImpedanceControlCommand.Request
        cartesian_impedance_control_command: _basic_command_pb2.CartesianImpedanceControlCommand.Request
        def __init__(self, command_header: _Optional[_Union[_command_header_pb2.CommandHeader.Request, _Mapping]] = ..., joint_position_command: _Optional[_Union[_basic_command_pb2.JointPositionCommand.Request, _Mapping]] = ..., optimal_control_command: _Optional[_Union[_basic_command_pb2.OptimalControlCommand.Request, _Mapping]] = ..., gravity_compensation_command: _Optional[_Union[_basic_command_pb2.GravityCompensationCommand.Request, _Mapping]] = ..., cartesian_command: _Optional[_Union[_basic_command_pb2.CartesianCommand.Request, _Mapping]] = ..., body_component_based_command: _Optional[_Union[_body_component_based_command_pb2.BodyComponentBasedCommand.Request, _Mapping]] = ..., joint_impedance_control_command: _Optional[_Union[_basic_command_pb2.JointImpedanceControlCommand.Request, _Mapping]] = ..., cartesian_impedance_control_command: _Optional[_Union[_basic_command_pb2.CartesianImpedanceControlCommand.Request, _Mapping]] = ...) -> None: ...
    class Feedback(_message.Message):
        __slots__ = ("command_header_feedback", "joint_position_command_feedback", "optimal_control_command_feedback", "gravity_compensation_command_feedback", "cartesian_command_feedback", "body_component_based_command_feedback", "joint_impedance_control_command_feedback", "cartesian_impedance_control_command_feedback")
        COMMAND_HEADER_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        JOINT_POSITION_COMMAND_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        OPTIMAL_CONTROL_COMMAND_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        GRAVITY_COMPENSATION_COMMAND_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        CARTESIAN_COMMAND_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        BODY_COMPONENT_BASED_COMMAND_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        JOINT_IMPEDANCE_CONTROL_COMMAND_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        CARTESIAN_IMPEDANCE_CONTROL_COMMAND_FEEDBACK_FIELD_NUMBER: _ClassVar[int]
        command_header_feedback: _command_header_pb2.CommandHeader.Feedback
        joint_position_command_feedback: _basic_command_pb2.JointPositionCommand.Feedback
        optimal_control_command_feedback: _basic_command_pb2.OptimalControlCommand.Feedback
        gravity_compensation_command_feedback: _basic_command_pb2.GravityCompensationCommand.Feedback
        cartesian_command_feedback: _basic_command_pb2.CartesianCommand.Feedback
        body_component_based_command_feedback: _body_component_based_command_pb2.BodyComponentBasedCommand.Feedback
        joint_impedance_control_command_feedback: _basic_command_pb2.JointImpedanceControlCommand.Feedback
        cartesian_impedance_control_command_feedback: _basic_command_pb2.CartesianImpedanceControlCommand.Feedback
        def __init__(self, command_header_feedback: _Optional[_Union[_command_header_pb2.CommandHeader.Feedback, _Mapping]] = ..., joint_position_command_feedback: _Optional[_Union[_basic_command_pb2.JointPositionCommand.Feedback, _Mapping]] = ..., optimal_control_command_feedback: _Optional[_Union[_basic_command_pb2.OptimalControlCommand.Feedback, _Mapping]] = ..., gravity_compensation_command_feedback: _Optional[_Union[_basic_command_pb2.GravityCompensationCommand.Feedback, _Mapping]] = ..., cartesian_command_feedback: _Optional[_Union[_basic_command_pb2.CartesianCommand.Feedback, _Mapping]] = ..., body_component_based_command_feedback: _Optional[_Union[_body_component_based_command_pb2.BodyComponentBasedCommand.Feedback, _Mapping]] = ..., joint_impedance_control_command_feedback: _Optional[_Union[_basic_command_pb2.JointImpedanceControlCommand.Feedback, _Mapping]] = ..., cartesian_impedance_control_command_feedback: _Optional[_Union[_basic_command_pb2.CartesianImpedanceControlCommand.Feedback, _Mapping]] = ...) -> None: ...
    def __init__(self) -> None: ...
