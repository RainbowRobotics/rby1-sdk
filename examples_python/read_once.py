import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'generated', 'python'))

import grpc
import asyncio

import rb.api.power_pb2 as power_pb2
import rb.api.power_service_pb2_grpc as power_service_pb2_grpc

import rb.api.robot_state_pb2 as robot_state_pb2
import rb.api.robot_state_service_pb2_grpc as robot_state_service_pb2_grpc

channel = grpc.insecure_channel('192.168.2.30')
power_service = power_service_pb2_grpc.PowerServiceStub(channel)
robot_state_service = robot_state_service_pb2_grpc.RobotStateServiceStub(channel)

# 

rv = power_service.PowerCommand(power_pb2.PowerCommandRequest(name="5v", 
                                                              command=power_pb2.PowerCommandRequest.COMMAND_POWER_ON))
print(rv)

rs = robot_state_service.GetRobotState(robot_state_pb2.GetRobotStateRequest())
print(rs)

