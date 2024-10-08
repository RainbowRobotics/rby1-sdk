# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc
import warnings

from rb.api import power_pb2 as rb_dot_api_dot_power__pb2

GRPC_GENERATED_VERSION = '1.65.2'
GRPC_VERSION = grpc.__version__
EXPECTED_ERROR_RELEASE = '1.66.0'
SCHEDULED_RELEASE_DATE = 'August 6, 2024'
_version_not_supported = False

try:
    from grpc._utilities import first_version_is_lower
    _version_not_supported = first_version_is_lower(GRPC_VERSION, GRPC_GENERATED_VERSION)
except ImportError:
    _version_not_supported = True

if _version_not_supported:
    warnings.warn(
        f'The grpc package installed is at version {GRPC_VERSION},'
        + f' but the generated code in rb/api/power_service_pb2_grpc.py depends on'
        + f' grpcio>={GRPC_GENERATED_VERSION}.'
        + f' Please upgrade your grpc module to grpcio>={GRPC_GENERATED_VERSION}'
        + f' or downgrade your generated code using grpcio-tools<={GRPC_VERSION}.'
        + f' This warning will become an error in {EXPECTED_ERROR_RELEASE},'
        + f' scheduled for release on {SCHEDULED_RELEASE_DATE}.',
        RuntimeWarning
    )


class PowerServiceStub(object):
    """Missing associated documentation comment in .proto file."""

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.PowerCommand = channel.unary_unary(
                '/rb.api.PowerService/PowerCommand',
                request_serializer=rb_dot_api_dot_power__pb2.PowerCommandRequest.SerializeToString,
                response_deserializer=rb_dot_api_dot_power__pb2.PowerCommandResponse.FromString,
                _registered_method=True)
        self.JointCommand = channel.unary_unary(
                '/rb.api.PowerService/JointCommand',
                request_serializer=rb_dot_api_dot_power__pb2.JointCommandRequest.SerializeToString,
                response_deserializer=rb_dot_api_dot_power__pb2.JointCommandResponse.FromString,
                _registered_method=True)
        self.ToolFlangePowerCommand = channel.unary_unary(
                '/rb.api.PowerService/ToolFlangePowerCommand',
                request_serializer=rb_dot_api_dot_power__pb2.ToolFlangePowerCommandRequest.SerializeToString,
                response_deserializer=rb_dot_api_dot_power__pb2.ToolFlangePowerCommandResponse.FromString,
                _registered_method=True)


class PowerServiceServicer(object):
    """Missing associated documentation comment in .proto file."""

    def PowerCommand(self, request, context):
        """Control power of the robot
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def JointCommand(self, request, context):
        """Joint command
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def ToolFlangePowerCommand(self, request, context):
        """Tool Flange
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_PowerServiceServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'PowerCommand': grpc.unary_unary_rpc_method_handler(
                    servicer.PowerCommand,
                    request_deserializer=rb_dot_api_dot_power__pb2.PowerCommandRequest.FromString,
                    response_serializer=rb_dot_api_dot_power__pb2.PowerCommandResponse.SerializeToString,
            ),
            'JointCommand': grpc.unary_unary_rpc_method_handler(
                    servicer.JointCommand,
                    request_deserializer=rb_dot_api_dot_power__pb2.JointCommandRequest.FromString,
                    response_serializer=rb_dot_api_dot_power__pb2.JointCommandResponse.SerializeToString,
            ),
            'ToolFlangePowerCommand': grpc.unary_unary_rpc_method_handler(
                    servicer.ToolFlangePowerCommand,
                    request_deserializer=rb_dot_api_dot_power__pb2.ToolFlangePowerCommandRequest.FromString,
                    response_serializer=rb_dot_api_dot_power__pb2.ToolFlangePowerCommandResponse.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'rb.api.PowerService', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))
    server.add_registered_method_handlers('rb.api.PowerService', rpc_method_handlers)


 # This class is part of an EXPERIMENTAL API.
class PowerService(object):
    """Missing associated documentation comment in .proto file."""

    @staticmethod
    def PowerCommand(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/rb.api.PowerService/PowerCommand',
            rb_dot_api_dot_power__pb2.PowerCommandRequest.SerializeToString,
            rb_dot_api_dot_power__pb2.PowerCommandResponse.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)

    @staticmethod
    def JointCommand(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/rb.api.PowerService/JointCommand',
            rb_dot_api_dot_power__pb2.JointCommandRequest.SerializeToString,
            rb_dot_api_dot_power__pb2.JointCommandResponse.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)

    @staticmethod
    def ToolFlangePowerCommand(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(
            request,
            target,
            '/rb.api.PowerService/ToolFlangePowerCommand',
            rb_dot_api_dot_power__pb2.ToolFlangePowerCommandRequest.SerializeToString,
            rb_dot_api_dot_power__pb2.ToolFlangePowerCommandResponse.FromString,
            options,
            channel_credentials,
            insecure,
            call_credentials,
            compression,
            wait_for_ready,
            timeout,
            metadata,
            _registered_method=True)
