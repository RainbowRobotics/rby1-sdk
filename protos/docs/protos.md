# Protocol Documentation
<a name="top"></a>

## Table of Contents

- [rb/api/arm_command.proto](#rb_api_arm_command-proto)
    - [ArmCommand](#rb-api-ArmCommand)
    - [ArmCommand.Feedback](#rb-api-ArmCommand-Feedback)
    - [ArmCommand.Request](#rb-api-ArmCommand-Request)
  
- [rb/api/basic_command.proto](#rb_api_basic_command-proto)
    - [CartesianCommand](#rb-api-CartesianCommand)
    - [CartesianCommand.Feedback](#rb-api-CartesianCommand-Feedback)
    - [CartesianCommand.Request](#rb-api-CartesianCommand-Request)
    - [CartesianCommand.SE3PoseTarget](#rb-api-CartesianCommand-SE3PoseTarget)
    - [CartesianCommand.TrackingError](#rb-api-CartesianCommand-TrackingError)
    - [GravityCompensationCommand](#rb-api-GravityCompensationCommand)
    - [GravityCompensationCommand.Feedback](#rb-api-GravityCompensationCommand-Feedback)
    - [GravityCompensationCommand.Request](#rb-api-GravityCompensationCommand-Request)
    - [ImpedanceControlCommand](#rb-api-ImpedanceControlCommand)
    - [ImpedanceControlCommand.Feedback](#rb-api-ImpedanceControlCommand-Feedback)
    - [ImpedanceControlCommand.Request](#rb-api-ImpedanceControlCommand-Request)
    - [ImpedanceControlCommand.TrackingError](#rb-api-ImpedanceControlCommand-TrackingError)
    - [JogCommand](#rb-api-JogCommand)
    - [JogCommand.Feedback](#rb-api-JogCommand-Feedback)
    - [JogCommand.Request](#rb-api-JogCommand-Request)
    - [JointPositionCommand](#rb-api-JointPositionCommand)
    - [JointPositionCommand.Feedback](#rb-api-JointPositionCommand-Feedback)
    - [JointPositionCommand.Request](#rb-api-JointPositionCommand-Request)
    - [JointVelocityCommand](#rb-api-JointVelocityCommand)
    - [JointVelocityCommand.Feedback](#rb-api-JointVelocityCommand-Feedback)
    - [JointVelocityCommand.Request](#rb-api-JointVelocityCommand-Request)
    - [OptimalControlCommand](#rb-api-OptimalControlCommand)
    - [OptimalControlCommand.CartesianCost](#rb-api-OptimalControlCommand-CartesianCost)
    - [OptimalControlCommand.CenterOfMassCost](#rb-api-OptimalControlCommand-CenterOfMassCost)
    - [OptimalControlCommand.Feedback](#rb-api-OptimalControlCommand-Feedback)
    - [OptimalControlCommand.JointPositionCost](#rb-api-OptimalControlCommand-JointPositionCost)
    - [OptimalControlCommand.Request](#rb-api-OptimalControlCommand-Request)
    - [RealTimeControlCommand](#rb-api-RealTimeControlCommand)
    - [RealTimeControlCommand.Feedback](#rb-api-RealTimeControlCommand-Feedback)
    - [RealTimeControlCommand.Request](#rb-api-RealTimeControlCommand-Request)
    - [SE2VelocityCommand](#rb-api-SE2VelocityCommand)
    - [SE2VelocityCommand.Feedback](#rb-api-SE2VelocityCommand-Feedback)
    - [SE2VelocityCommand.Request](#rb-api-SE2VelocityCommand-Request)
    - [StopCommand](#rb-api-StopCommand)
    - [StopCommand.Feedback](#rb-api-StopCommand-Feedback)
    - [StopCommand.Request](#rb-api-StopCommand-Request)
  
- [rb/api/body_command.proto](#rb_api_body_command-proto)
    - [BodyCommand](#rb-api-BodyCommand)
    - [BodyCommand.Feedback](#rb-api-BodyCommand-Feedback)
    - [BodyCommand.Request](#rb-api-BodyCommand-Request)
  
- [rb/api/body_component_based_command.proto](#rb_api_body_component_based_command-proto)
    - [BodyComponentBasedCommand](#rb-api-BodyComponentBasedCommand)
    - [BodyComponentBasedCommand.Feedback](#rb-api-BodyComponentBasedCommand-Feedback)
    - [BodyComponentBasedCommand.Request](#rb-api-BodyComponentBasedCommand-Request)
  
- [rb/api/command_header.proto](#rb_api_command_header-proto)
    - [CommandHeader](#rb-api-CommandHeader)
    - [CommandHeader.Feedback](#rb-api-CommandHeader-Feedback)
    - [CommandHeader.Request](#rb-api-CommandHeader-Request)
    - [CommandHeader.Request.InertialsEntry](#rb-api-CommandHeader-Request-InertialsEntry)
  
- [rb/api/component_based_command.proto](#rb_api_component_based_command-proto)
    - [ComponentBasedCommand](#rb-api-ComponentBasedCommand)
    - [ComponentBasedCommand.Feedback](#rb-api-ComponentBasedCommand-Feedback)
    - [ComponentBasedCommand.Request](#rb-api-ComponentBasedCommand-Request)
  
- [rb/api/control_manager.proto](#rb_api_control_manager-proto)
    - [ControlManagerCommandRequest](#rb-api-ControlManagerCommandRequest)
    - [ControlManagerCommandResponse](#rb-api-ControlManagerCommandResponse)
    - [ControlManagerState](#rb-api-ControlManagerState)
    - [GetTimeScaleRequest](#rb-api-GetTimeScaleRequest)
    - [GetTimeScaleResponse](#rb-api-GetTimeScaleResponse)
    - [SetTimeScaleRequest](#rb-api-SetTimeScaleRequest)
    - [SetTimeScaleResponse](#rb-api-SetTimeScaleResponse)
  
    - [ControlManagerCommandRequest.Command](#rb-api-ControlManagerCommandRequest-Command)
    - [ControlManagerState.ControlState](#rb-api-ControlManagerState-ControlState)
    - [ControlManagerState.State](#rb-api-ControlManagerState-State)
  
- [rb/api/control_manager_service.proto](#rb_api_control_manager_service-proto)
    - [ControlManagerService](#rb-api-ControlManagerService)
  
- [rb/api/gamepad.proto](#rb_api_gamepad-proto)
    - [Gamepad](#rb-api-Gamepad)
    - [UploadGamepadDataRequest](#rb-api-UploadGamepadDataRequest)
    - [UploadGamepadDataResponse](#rb-api-UploadGamepadDataResponse)
  
- [rb/api/gamepad_service.proto](#rb_api_gamepad_service-proto)
    - [GamepadService](#rb-api-GamepadService)
  
- [rb/api/geometry.proto](#rb_api_geometry-proto)
    - [EulerAngleZYX](#rb-api-EulerAngleZYX)
    - [Inertia](#rb-api-Inertia)
    - [Inertial](#rb-api-Inertial)
    - [Quaternion](#rb-api-Quaternion)
    - [SE2Pose](#rb-api-SE2Pose)
    - [SE2Velocity](#rb-api-SE2Velocity)
    - [SE3Pose](#rb-api-SE3Pose)
    - [Vec2](#rb-api-Vec2)
    - [Vec3](#rb-api-Vec3)
  
- [rb/api/gripper_command.proto](#rb_api_gripper_command-proto)
    - [GripperInitializationRequest](#rb-api-GripperInitializationRequest)
    - [GripperInitializationResponse](#rb-api-GripperInitializationResponse)
    - [GripperMoveRequest](#rb-api-GripperMoveRequest)
    - [GripperMoveResponse](#rb-api-GripperMoveResponse)
  
- [rb/api/gripper_command_service.proto](#rb_api_gripper_command_service-proto)
    - [GripperCommandService](#rb-api-GripperCommandService)
  
- [rb/api/head_command.proto](#rb_api_head_command-proto)
    - [HeadCommand](#rb-api-HeadCommand)
    - [HeadCommand.Feedback](#rb-api-HeadCommand-Feedback)
    - [HeadCommand.Request](#rb-api-HeadCommand-Request)
  
- [rb/api/header.proto](#rb_api_header-proto)
    - [CommonError](#rb-api-CommonError)
    - [RequestHeader](#rb-api-RequestHeader)
    - [ResponseHeader](#rb-api-ResponseHeader)
  
    - [CommonError.Code](#rb-api-CommonError-Code)
  
- [rb/api/log.proto](#rb_api_log-proto)
    - [GetLastLogRequest](#rb-api-GetLastLogRequest)
    - [GetLastLogResponse](#rb-api-GetLastLogResponse)
    - [GetLogStreamRequest](#rb-api-GetLogStreamRequest)
    - [GetLogStreamResponse](#rb-api-GetLogStreamResponse)
    - [Log](#rb-api-Log)
    - [SetLogLevelRequest](#rb-api-SetLogLevelRequest)
    - [SetLogLevelResponse](#rb-api-SetLogLevelResponse)
  
    - [Log.Level](#rb-api-Log-Level)
  
- [rb/api/log_service.proto](#rb_api_log_service-proto)
    - [LogService](#rb-api-LogService)
  
- [rb/api/mobility_command.proto](#rb_api_mobility_command-proto)
    - [MobilityCommand](#rb-api-MobilityCommand)
    - [MobilityCommand.Feedback](#rb-api-MobilityCommand-Feedback)
    - [MobilityCommand.Request](#rb-api-MobilityCommand-Request)
  
- [rb/api/parameter.proto](#rb_api_parameter-proto)
    - [GetParameterListRequest](#rb-api-GetParameterListRequest)
    - [GetParameterListResponse](#rb-api-GetParameterListResponse)
    - [GetParameterListResponse.ParameterType](#rb-api-GetParameterListResponse-ParameterType)
    - [GetParameterRequest](#rb-api-GetParameterRequest)
    - [GetParameterResponse](#rb-api-GetParameterResponse)
    - [ResetAllParametersToDefaultRequest](#rb-api-ResetAllParametersToDefaultRequest)
    - [ResetAllParametersToDefaultResponse](#rb-api-ResetAllParametersToDefaultResponse)
    - [ResetParameterToDefaultRequest](#rb-api-ResetParameterToDefaultRequest)
    - [ResetParameterToDefaultResponse](#rb-api-ResetParameterToDefaultResponse)
    - [SetParameterRequest](#rb-api-SetParameterRequest)
    - [SetParameterResponse](#rb-api-SetParameterResponse)
  
- [rb/api/parameter_service.proto](#rb_api_parameter_service-proto)
    - [ParameterService](#rb-api-ParameterService)
  
- [rb/api/ping.proto](#rb_api_ping-proto)
    - [PingRequest](#rb-api-PingRequest)
    - [PingResponse](#rb-api-PingResponse)
  
- [rb/api/ping_service.proto](#rb_api_ping_service-proto)
    - [PingService](#rb-api-PingService)
  
- [rb/api/power.proto](#rb_api_power-proto)
    - [JointCommandRequest](#rb-api-JointCommandRequest)
    - [JointCommandResponse](#rb-api-JointCommandResponse)
    - [PowerCommandRequest](#rb-api-PowerCommandRequest)
    - [PowerCommandResponse](#rb-api-PowerCommandResponse)
    - [ToolFlangePowerCommandRequest](#rb-api-ToolFlangePowerCommandRequest)
    - [ToolFlangePowerCommandResponse](#rb-api-ToolFlangePowerCommandResponse)
  
    - [JointCommandRequest.Command](#rb-api-JointCommandRequest-Command)
    - [JointCommandResponse.Status](#rb-api-JointCommandResponse-Status)
    - [PowerCommandRequest.Command](#rb-api-PowerCommandRequest-Command)
    - [PowerCommandResponse.Status](#rb-api-PowerCommandResponse-Status)
    - [ToolFlangePowerCommandRequest.Command](#rb-api-ToolFlangePowerCommandRequest-Command)
  
- [rb/api/power_service.proto](#rb_api_power_service-proto)
    - [PowerService](#rb-api-PowerService)
  
- [rb/api/robot_command.proto](#rb_api_robot_command-proto)
    - [RobotCommand](#rb-api-RobotCommand)
    - [RobotCommand.Feedback](#rb-api-RobotCommand-Feedback)
    - [RobotCommand.Request](#rb-api-RobotCommand-Request)
    - [RobotCommandRequest](#rb-api-RobotCommandRequest)
    - [RobotCommandResponse](#rb-api-RobotCommandResponse)
  
    - [RobotCommand.Feedback.FinishCode](#rb-api-RobotCommand-Feedback-FinishCode)
    - [RobotCommand.Feedback.Status](#rb-api-RobotCommand-Feedback-Status)
  
- [rb/api/robot_command_service.proto](#rb_api_robot_command_service-proto)
    - [RobotCommandService](#rb-api-RobotCommandService)
  
- [rb/api/robot_info.proto](#rb_api_robot_info-proto)
    - [BatteryInfo](#rb-api-BatteryInfo)
    - [EMOInfo](#rb-api-EMOInfo)
    - [GetRobotInfoRequest](#rb-api-GetRobotInfoRequest)
    - [GetRobotInfoResponse](#rb-api-GetRobotInfoResponse)
    - [GetRobotModelRequest](#rb-api-GetRobotModelRequest)
    - [GetRobotModelResponse](#rb-api-GetRobotModelResponse)
    - [ImportRobotModelRequest](#rb-api-ImportRobotModelRequest)
    - [ImportRobotModelResponse](#rb-api-ImportRobotModelResponse)
    - [JointInfo](#rb-api-JointInfo)
    - [PowerInfo](#rb-api-PowerInfo)
    - [RobotInfo](#rb-api-RobotInfo)
  
- [rb/api/robot_info_service.proto](#rb_api_robot_info_service-proto)
    - [RobotInfoService](#rb-api-RobotInfoService)
  
- [rb/api/robot_state.proto](#rb_api_robot_state-proto)
    - [BatteryState](#rb-api-BatteryState)
    - [Collision](#rb-api-Collision)
    - [EMOState](#rb-api-EMOState)
    - [FTSensorData](#rb-api-FTSensorData)
    - [GetControlManagerStateRequest](#rb-api-GetControlManagerStateRequest)
    - [GetControlManagerStateResponse](#rb-api-GetControlManagerStateResponse)
    - [GetRobotStateRequest](#rb-api-GetRobotStateRequest)
    - [GetRobotStateResponse](#rb-api-GetRobotStateResponse)
    - [GetRobotStateStreamRequest](#rb-api-GetRobotStateStreamRequest)
    - [GetRobotStateStreamResponse](#rb-api-GetRobotStateStreamResponse)
    - [JointState](#rb-api-JointState)
    - [PowerState](#rb-api-PowerState)
    - [ResetOdometryRequest](#rb-api-ResetOdometryRequest)
    - [ResetOdometryResponse](#rb-api-ResetOdometryResponse)
    - [RobotState](#rb-api-RobotState)
    - [SystemStat](#rb-api-SystemStat)
    - [ToolFlangeState](#rb-api-ToolFlangeState)
  
    - [EMOState.State](#rb-api-EMOState-State)
    - [JointState.FETState](#rb-api-JointState-FETState)
    - [JointState.InitializationState](#rb-api-JointState-InitializationState)
    - [JointState.RunState](#rb-api-JointState-RunState)
    - [PowerState.State](#rb-api-PowerState-State)
  
- [rb/api/robot_state_service.proto](#rb_api_robot_state_service-proto)
    - [RobotStateService](#rb-api-RobotStateService)
  
- [rb/api/torso_command.proto](#rb_api_torso_command-proto)
    - [TorsoCommand](#rb-api-TorsoCommand)
    - [TorsoCommand.Feedback](#rb-api-TorsoCommand-Feedback)
    - [TorsoCommand.Request](#rb-api-TorsoCommand-Request)
  
- [rb/api/whole_body_command.proto](#rb_api_whole_body_command-proto)
    - [WholeBodyCommand](#rb-api-WholeBodyCommand)
    - [WholeBodyCommand.Feedback](#rb-api-WholeBodyCommand-Feedback)
    - [WholeBodyCommand.Request](#rb-api-WholeBodyCommand-Request)
  
- [Scalar Value Types](#scalar-value-types)



<a name="rb_api_arm_command-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/arm_command.proto



<a name="rb-api-ArmCommand"></a>

### ArmCommand







<a name="rb-api-ArmCommand-Feedback"></a>

### ArmCommand.Feedback



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header_feedback | [CommandHeader.Feedback](#rb-api-CommandHeader-Feedback) |  |  |
| joint_position_command_feedback | [JointPositionCommand.Feedback](#rb-api-JointPositionCommand-Feedback) |  |  |
| gravity_compensation_command_feedback | [GravityCompensationCommand.Feedback](#rb-api-GravityCompensationCommand-Feedback) |  |  |
| cartesian_command_feedback | [CartesianCommand.Feedback](#rb-api-CartesianCommand-Feedback) |  |  |
| impedance_control_command_feedback | [ImpedanceControlCommand.Feedback](#rb-api-ImpedanceControlCommand-Feedback) |  |  |






<a name="rb-api-ArmCommand-Request"></a>

### ArmCommand.Request



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header | [CommandHeader.Request](#rb-api-CommandHeader-Request) |  |  |
| joint_position_command | [JointPositionCommand.Request](#rb-api-JointPositionCommand-Request) |  |  |
| gravity_compensation_command | [GravityCompensationCommand.Request](#rb-api-GravityCompensationCommand-Request) |  |  |
| cartesian_command | [CartesianCommand.Request](#rb-api-CartesianCommand-Request) |  |  |
| impedance_control_command | [ImpedanceControlCommand.Request](#rb-api-ImpedanceControlCommand-Request) |  |  |





 

 

 

 



<a name="rb_api_basic_command-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/basic_command.proto



<a name="rb-api-CartesianCommand"></a>

### CartesianCommand







<a name="rb-api-CartesianCommand-Feedback"></a>

### CartesianCommand.Feedback



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header_feedback | [CommandHeader.Feedback](#rb-api-CommandHeader-Feedback) |  |  |
| tracking_errors | [CartesianCommand.TrackingError](#rb-api-CartesianCommand-TrackingError) | repeated |  |






<a name="rb-api-CartesianCommand-Request"></a>

### CartesianCommand.Request



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header | [CommandHeader.Request](#rb-api-CommandHeader-Request) |  |  |
| minimum_time | [google.protobuf.Duration](#google-protobuf-Duration) |  |  |
| targets | [CartesianCommand.SE3PoseTarget](#rb-api-CartesianCommand-SE3PoseTarget) | repeated |  |
| stop_position_tracking_error | [google.protobuf.DoubleValue](#google-protobuf-DoubleValue) |  |  |
| stop_orientation_tracking_error | [google.protobuf.DoubleValue](#google-protobuf-DoubleValue) |  |  |






<a name="rb-api-CartesianCommand-SE3PoseTarget"></a>

### CartesianCommand.SE3PoseTarget



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| ref_link_name | [string](#string) |  |  |
| link_name | [string](#string) |  |  |
| T | [SE3Pose](#rb-api-SE3Pose) |  |  |
| linear_velocity_limit | [google.protobuf.DoubleValue](#google-protobuf-DoubleValue) |  | (m/s) |
| angular_velocity_limit | [google.protobuf.DoubleValue](#google-protobuf-DoubleValue) |  | (rad/s) |
| acceleration_limit_scaling | [google.protobuf.DoubleValue](#google-protobuf-DoubleValue) |  | default.linear_acceleration_limit * acceleration_limit_scaling default.angular_acceleration_limit * acceleration_limit_scaling

`(0, 1]` |






<a name="rb-api-CartesianCommand-TrackingError"></a>

### CartesianCommand.TrackingError



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| position_error | [double](#double) |  |  |
| rotation_error | [double](#double) |  |  |






<a name="rb-api-GravityCompensationCommand"></a>

### GravityCompensationCommand







<a name="rb-api-GravityCompensationCommand-Feedback"></a>

### GravityCompensationCommand.Feedback



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header_feedback | [CommandHeader.Feedback](#rb-api-CommandHeader-Feedback) |  |  |






<a name="rb-api-GravityCompensationCommand-Request"></a>

### GravityCompensationCommand.Request



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header | [CommandHeader.Request](#rb-api-CommandHeader-Request) |  |  |
| on | [bool](#bool) |  |  |






<a name="rb-api-ImpedanceControlCommand"></a>

### ImpedanceControlCommand







<a name="rb-api-ImpedanceControlCommand-Feedback"></a>

### ImpedanceControlCommand.Feedback



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header_feedback | [CommandHeader.Feedback](#rb-api-CommandHeader-Feedback) |  |  |
| tracking_error | [ImpedanceControlCommand.TrackingError](#rb-api-ImpedanceControlCommand-TrackingError) |  |  |






<a name="rb-api-ImpedanceControlCommand-Request"></a>

### ImpedanceControlCommand.Request



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header | [CommandHeader.Request](#rb-api-CommandHeader-Request) |  |  |
| ref_link_name | [string](#string) |  |  |
| link_name | [string](#string) |  |  |
| T | [SE3Pose](#rb-api-SE3Pose) |  |  |
| translation_weight | [Vec3](#rb-api-Vec3) |  |  |
| rotation_weight | [Vec3](#rb-api-Vec3) |  |  |






<a name="rb-api-ImpedanceControlCommand-TrackingError"></a>

### ImpedanceControlCommand.TrackingError



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| position_error | [double](#double) |  |  |
| rotation_error | [double](#double) |  |  |






<a name="rb-api-JogCommand"></a>

### JogCommand







<a name="rb-api-JogCommand-Feedback"></a>

### JogCommand.Feedback



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header_feedback | [CommandHeader.Feedback](#rb-api-CommandHeader-Feedback) |  |  |
| target_joint_name | [string](#string) |  |  |






<a name="rb-api-JogCommand-Request"></a>

### JogCommand.Request



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header | [CommandHeader.Request](#rb-api-CommandHeader-Request) |  |  |
| joint_name | [string](#string) |  |  |
| velocity_limit | [google.protobuf.DoubleValue](#google-protobuf-DoubleValue) |  | (rad/s) (optional) |
| acceleration_limit | [google.protobuf.DoubleValue](#google-protobuf-DoubleValue) |  | (rad/s^2) (optional) |
| absolute_position | [double](#double) |  | (rad) |
| relative_position | [double](#double) |  | (rad) current position &#43; relative position |
| one_step | [bool](#bool) |  | 5 deg, true is positive move, false is negative move |






<a name="rb-api-JointPositionCommand"></a>

### JointPositionCommand







<a name="rb-api-JointPositionCommand-Feedback"></a>

### JointPositionCommand.Feedback



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header_feedback | [CommandHeader.Feedback](#rb-api-CommandHeader-Feedback) |  |  |






<a name="rb-api-JointPositionCommand-Request"></a>

### JointPositionCommand.Request



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header | [CommandHeader.Request](#rb-api-CommandHeader-Request) |  |  |
| minimum_time | [google.protobuf.Duration](#google-protobuf-Duration) |  |  |
| position | [double](#double) | repeated |  |
| velocity_limit | [double](#double) | repeated |  |
| acceleration_limit | [double](#double) | repeated |  |
| cutoff_frequency | [google.protobuf.DoubleValue](#google-protobuf-DoubleValue) |  |  |






<a name="rb-api-JointVelocityCommand"></a>

### JointVelocityCommand







<a name="rb-api-JointVelocityCommand-Feedback"></a>

### JointVelocityCommand.Feedback



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header_feedback | [CommandHeader.Feedback](#rb-api-CommandHeader-Feedback) |  |  |






<a name="rb-api-JointVelocityCommand-Request"></a>

### JointVelocityCommand.Request



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header | [CommandHeader.Request](#rb-api-CommandHeader-Request) |  |  |
| minimum_time | [google.protobuf.Duration](#google-protobuf-Duration) |  |  |
| velocity | [double](#double) | repeated |  |
| acceleration_limit | [double](#double) | repeated |  |






<a name="rb-api-OptimalControlCommand"></a>

### OptimalControlCommand







<a name="rb-api-OptimalControlCommand-CartesianCost"></a>

### OptimalControlCommand.CartesianCost



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| ref_link_name | [string](#string) |  |  |
| link_name | [string](#string) |  |  |
| T | [SE3Pose](#rb-api-SE3Pose) |  |  |
| translation_weight | [double](#double) |  | default = 1 |
| rotation_weight | [double](#double) |  | default = 1 |






<a name="rb-api-OptimalControlCommand-CenterOfMassCost"></a>

### OptimalControlCommand.CenterOfMassCost



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| ref_link_name | [string](#string) |  |  |
| pose | [Vec3](#rb-api-Vec3) |  |  |
| weight | [double](#double) |  | default = 1 |






<a name="rb-api-OptimalControlCommand-Feedback"></a>

### OptimalControlCommand.Feedback



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header_feedback | [CommandHeader.Feedback](#rb-api-CommandHeader-Feedback) |  |  |
| total_cost | [double](#double) |  |  |
| cartesian_costs | [double](#double) | repeated |  |
| center_of_mass_cost | [double](#double) |  |  |
| joint_position_costs | [double](#double) | repeated |  |






<a name="rb-api-OptimalControlCommand-JointPositionCost"></a>

### OptimalControlCommand.JointPositionCost



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| joint_name | [string](#string) |  |  |
| target_position | [double](#double) |  |  |
| weight | [double](#double) |  |  |






<a name="rb-api-OptimalControlCommand-Request"></a>

### OptimalControlCommand.Request



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header | [CommandHeader.Request](#rb-api-CommandHeader-Request) |  |  |
| cartesian_costs | [OptimalControlCommand.CartesianCost](#rb-api-OptimalControlCommand-CartesianCost) | repeated |  |
| center_of_mass_cost | [OptimalControlCommand.CenterOfMassCost](#rb-api-OptimalControlCommand-CenterOfMassCost) |  |  |
| joint_position_costs | [OptimalControlCommand.JointPositionCost](#rb-api-OptimalControlCommand-JointPositionCost) | repeated |  |
| velocity_limit_scaling | [google.protobuf.DoubleValue](#google-protobuf-DoubleValue) |  | velocity scaling factor: qdot_limit * default.velocity_limit_scaling * velocity_limit_scaling default: 1.0, range: (0, 1] |
| velocity_tracking_gain | [google.protobuf.DoubleValue](#google-protobuf-DoubleValue) |  | velocity tracking gain default: default.optimal_control_command.velocity_tracking_gain, range: (0, 1] |
| stop_cost | [google.protobuf.DoubleValue](#google-protobuf-DoubleValue) |  | stop cost default: default.optimal_control_command.stop_cost, range: (0, ∞) |
| min_delta_cost | [google.protobuf.DoubleValue](#google-protobuf-DoubleValue) |  | minimum delta cost range: (0, ∞) |
| patience | [google.protobuf.Int32Value](#google-protobuf-Int32Value) |  | patience parameter range: (0, ∞) |






<a name="rb-api-RealTimeControlCommand"></a>

### RealTimeControlCommand







<a name="rb-api-RealTimeControlCommand-Feedback"></a>

### RealTimeControlCommand.Feedback



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header_feedback | [CommandHeader.Feedback](#rb-api-CommandHeader-Feedback) |  |  |






<a name="rb-api-RealTimeControlCommand-Request"></a>

### RealTimeControlCommand.Request



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header | [CommandHeader.Request](#rb-api-CommandHeader-Request) |  |  |
| port | [uint32](#uint32) |  |  |






<a name="rb-api-SE2VelocityCommand"></a>

### SE2VelocityCommand







<a name="rb-api-SE2VelocityCommand-Feedback"></a>

### SE2VelocityCommand.Feedback



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header_feedback | [CommandHeader.Feedback](#rb-api-CommandHeader-Feedback) |  |  |






<a name="rb-api-SE2VelocityCommand-Request"></a>

### SE2VelocityCommand.Request



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header | [CommandHeader.Request](#rb-api-CommandHeader-Request) |  |  |
| minimum_time | [google.protobuf.Duration](#google-protobuf-Duration) |  |  |
| velocity | [SE2Velocity](#rb-api-SE2Velocity) |  |  |
| acceleration_limit | [SE2Velocity](#rb-api-SE2Velocity) |  |  |






<a name="rb-api-StopCommand"></a>

### StopCommand







<a name="rb-api-StopCommand-Feedback"></a>

### StopCommand.Feedback



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header_feedback | [CommandHeader.Feedback](#rb-api-CommandHeader-Feedback) |  |  |






<a name="rb-api-StopCommand-Request"></a>

### StopCommand.Request



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header | [CommandHeader.Request](#rb-api-CommandHeader-Request) |  |  |





 

 

 

 



<a name="rb_api_body_command-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/body_command.proto



<a name="rb-api-BodyCommand"></a>

### BodyCommand







<a name="rb-api-BodyCommand-Feedback"></a>

### BodyCommand.Feedback



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header_feedback | [CommandHeader.Feedback](#rb-api-CommandHeader-Feedback) |  |  |
| joint_position_command_feedback | [JointPositionCommand.Feedback](#rb-api-JointPositionCommand-Feedback) |  |  |
| optimal_control_command_feedback | [OptimalControlCommand.Feedback](#rb-api-OptimalControlCommand-Feedback) |  |  |
| gravity_compensation_command_feedback | [GravityCompensationCommand.Feedback](#rb-api-GravityCompensationCommand-Feedback) |  |  |
| cartesian_command_feedback | [CartesianCommand.Feedback](#rb-api-CartesianCommand-Feedback) |  |  |
| body_component_based_command_feedback | [BodyComponentBasedCommand.Feedback](#rb-api-BodyComponentBasedCommand-Feedback) |  |  |






<a name="rb-api-BodyCommand-Request"></a>

### BodyCommand.Request



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header | [CommandHeader.Request](#rb-api-CommandHeader-Request) |  |  |
| joint_position_command | [JointPositionCommand.Request](#rb-api-JointPositionCommand-Request) |  |  |
| optimal_control_command | [OptimalControlCommand.Request](#rb-api-OptimalControlCommand-Request) |  |  |
| gravity_compensation_command | [GravityCompensationCommand.Request](#rb-api-GravityCompensationCommand-Request) |  |  |
| cartesian_command | [CartesianCommand.Request](#rb-api-CartesianCommand-Request) |  |  |
| body_component_based_command | [BodyComponentBasedCommand.Request](#rb-api-BodyComponentBasedCommand-Request) |  |  |





 

 

 

 



<a name="rb_api_body_component_based_command-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/body_component_based_command.proto



<a name="rb-api-BodyComponentBasedCommand"></a>

### BodyComponentBasedCommand







<a name="rb-api-BodyComponentBasedCommand-Feedback"></a>

### BodyComponentBasedCommand.Feedback



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header_feedback | [CommandHeader.Feedback](#rb-api-CommandHeader-Feedback) |  |  |
| right_arm_command_feedback | [ArmCommand.Feedback](#rb-api-ArmCommand-Feedback) |  |  |
| left_arm_command_feedback | [ArmCommand.Feedback](#rb-api-ArmCommand-Feedback) |  |  |
| torso_command_feedback | [TorsoCommand.Feedback](#rb-api-TorsoCommand-Feedback) |  |  |






<a name="rb-api-BodyComponentBasedCommand-Request"></a>

### BodyComponentBasedCommand.Request



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header | [CommandHeader.Request](#rb-api-CommandHeader-Request) |  |  |
| right_arm_command | [ArmCommand.Request](#rb-api-ArmCommand-Request) |  |  |
| left_arm_command | [ArmCommand.Request](#rb-api-ArmCommand-Request) |  |  |
| torso_command | [TorsoCommand.Request](#rb-api-TorsoCommand-Request) |  |  |





 

 

 

 



<a name="rb_api_command_header-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/command_header.proto



<a name="rb-api-CommandHeader"></a>

### CommandHeader







<a name="rb-api-CommandHeader-Feedback"></a>

### CommandHeader.Feedback



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| finished | [bool](#bool) |  |  |






<a name="rb-api-CommandHeader-Request"></a>

### CommandHeader.Request



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| control_hold_time | [google.protobuf.Duration](#google-protobuf-Duration) |  |  |
| gravity | [Vec3](#rb-api-Vec3) |  |  |
| inertials | [CommandHeader.Request.InertialsEntry](#rb-api-CommandHeader-Request-InertialsEntry) | repeated |  |






<a name="rb-api-CommandHeader-Request-InertialsEntry"></a>

### CommandHeader.Request.InertialsEntry



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| key | [string](#string) |  |  |
| value | [Inertial](#rb-api-Inertial) |  |  |





 

 

 

 



<a name="rb_api_component_based_command-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/component_based_command.proto



<a name="rb-api-ComponentBasedCommand"></a>

### ComponentBasedCommand







<a name="rb-api-ComponentBasedCommand-Feedback"></a>

### ComponentBasedCommand.Feedback



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header_feedback | [CommandHeader.Feedback](#rb-api-CommandHeader-Feedback) |  |  |
| mobility_command_feedback | [MobilityCommand.Feedback](#rb-api-MobilityCommand-Feedback) |  |  |
| body_command_feedback | [BodyCommand.Feedback](#rb-api-BodyCommand-Feedback) |  |  |
| head_command_feedback | [HeadCommand.Feedback](#rb-api-HeadCommand-Feedback) |  |  |






<a name="rb-api-ComponentBasedCommand-Request"></a>

### ComponentBasedCommand.Request



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header | [CommandHeader.Request](#rb-api-CommandHeader-Request) |  |  |
| mobility_command | [MobilityCommand.Request](#rb-api-MobilityCommand-Request) |  |  |
| body_command | [BodyCommand.Request](#rb-api-BodyCommand-Request) |  |  |
| head_command | [HeadCommand.Request](#rb-api-HeadCommand-Request) |  |  |





 

 

 

 



<a name="rb_api_control_manager-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/control_manager.proto



<a name="rb-api-ControlManagerCommandRequest"></a>

### ControlManagerCommandRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  | Request header |
| command | [ControlManagerCommandRequest.Command](#rb-api-ControlManagerCommandRequest-Command) |  |  |
| unlimited_mode_enabled | [google.protobuf.BoolValue](#google-protobuf-BoolValue) |  |  |






<a name="rb-api-ControlManagerCommandResponse"></a>

### ControlManagerCommandResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  | Response header |
| control_manager_state | [ControlManagerState](#rb-api-ControlManagerState) |  |  |






<a name="rb-api-ControlManagerState"></a>

### ControlManagerState



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| state | [ControlManagerState.State](#rb-api-ControlManagerState-State) |  |  |
| time_scale | [double](#double) |  |  |
| control_state | [ControlManagerState.ControlState](#rb-api-ControlManagerState-ControlState) |  |  |
| enabled_joint_idx | [uint32](#uint32) | repeated |  |
| unlimited_mode_enabled | [bool](#bool) |  |  |






<a name="rb-api-GetTimeScaleRequest"></a>

### GetTimeScaleRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  | Request header |






<a name="rb-api-GetTimeScaleResponse"></a>

### GetTimeScaleResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  | Response header |
| time_scale | [double](#double) |  |  |






<a name="rb-api-SetTimeScaleRequest"></a>

### SetTimeScaleRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  | Request header |
| time_scale | [double](#double) |  |  |






<a name="rb-api-SetTimeScaleResponse"></a>

### SetTimeScaleResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  | Response header |
| current_time_scale | [double](#double) |  |  |





 


<a name="rb-api-ControlManagerCommandRequest-Command"></a>

### ControlManagerCommandRequest.Command
Control manager command

| Name | Number | Description |
| ---- | ------ | ----------- |
| COMMAND_UNKNOWN | 0 |  |
| COMMAND_ENABLE | 1 |  |
| COMMAND_DISABLE | 2 |  |
| COMMAND_RESET_FAULT | 3 |  |



<a name="rb-api-ControlManagerState-ControlState"></a>

### ControlManagerState.ControlState


| Name | Number | Description |
| ---- | ------ | ----------- |
| CONTROL_STATE_UNKNOWN | 0 |  |
| CONTROL_STATE_IDLE | 1 |  |
| CONTROL_STATE_EXECUTING | 2 |  |
| CONTROL_STATE_SWITCHING | 3 |  |



<a name="rb-api-ControlManagerState-State"></a>

### ControlManagerState.State


| Name | Number | Description |
| ---- | ------ | ----------- |
| CONTROL_MANAGER_STATE_UNKNOWN | 0 |  |
| CONTROL_MANAGER_STATE_IDLE | 1 |  |
| CONTROL_MANAGER_STATE_ENABLED | 2 |  |
| CONTROL_MANAGER_STATE_MINOR_FAULT | 3 |  |
| CONTROL_MANAGER_STATE_MAJOR_FAULT | 4 |  |


 

 

 



<a name="rb_api_control_manager_service-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/control_manager_service.proto


 

 

 


<a name="rb-api-ControlManagerService"></a>

### ControlManagerService


| Method Name | Request Type | Response Type | Description |
| ----------- | ------------ | ------------- | ------------|
| ControlManagerCommand | [ControlManagerCommandRequest](#rb-api-ControlManagerCommandRequest) | [ControlManagerCommandResponse](#rb-api-ControlManagerCommandResponse) |  |
| GetTimeScale | [GetTimeScaleRequest](#rb-api-GetTimeScaleRequest) | [GetTimeScaleResponse](#rb-api-GetTimeScaleResponse) |  |
| SetTimeScale | [SetTimeScaleRequest](#rb-api-SetTimeScaleRequest) | [SetTimeScaleResponse](#rb-api-SetTimeScaleResponse) |  |

 



<a name="rb_api_gamepad-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/gamepad.proto



<a name="rb-api-Gamepad"></a>

### Gamepad



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| buttons | [bool](#bool) | repeated |  |
| joystick | [double](#double) | repeated |  |






<a name="rb-api-UploadGamepadDataRequest"></a>

### UploadGamepadDataRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  |  |
| data | [Gamepad](#rb-api-Gamepad) |  |  |






<a name="rb-api-UploadGamepadDataResponse"></a>

### UploadGamepadDataResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  |  |





 

 

 

 



<a name="rb_api_gamepad_service-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/gamepad_service.proto


 

 

 


<a name="rb-api-GamepadService"></a>

### GamepadService


| Method Name | Request Type | Response Type | Description |
| ----------- | ------------ | ------------- | ------------|
| UploadGamepadData | [Gamepad](#rb-api-Gamepad) stream | [UploadGamepadDataResponse](#rb-api-UploadGamepadDataResponse) |  |

 



<a name="rb_api_geometry-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/geometry.proto



<a name="rb-api-EulerAngleZYX"></a>

### EulerAngleZYX



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| z | [double](#double) |  |  |
| y | [double](#double) |  |  |
| x | [double](#double) |  |  |






<a name="rb-api-Inertia"></a>

### Inertia
Inertia tensor components (kg*m^2)


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| ixx | [double](#double) |  |  |
| iyy | [double](#double) |  |  |
| izz | [double](#double) |  |  |
| ixy | [double](#double) |  |  |
| ixz | [double](#double) |  |  |
| iyz | [double](#double) |  |  |






<a name="rb-api-Inertial"></a>

### Inertial



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| mass | [double](#double) |  | Mass (kg) |
| center_of_mass | [Vec3](#rb-api-Vec3) |  | Center of mass (m) |
| inertia | [Inertia](#rb-api-Inertia) |  | Inertia tensor |






<a name="rb-api-Quaternion"></a>

### Quaternion



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| x | [double](#double) |  |  |
| y | [double](#double) |  |  |
| z | [double](#double) |  |  |
| w | [double](#double) |  |  |






<a name="rb-api-SE2Pose"></a>

### SE2Pose



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| position | [Vec2](#rb-api-Vec2) |  | (m) |
| angle | [double](#double) |  | (rad) |






<a name="rb-api-SE2Velocity"></a>

### SE2Velocity



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| linear | [Vec2](#rb-api-Vec2) |  | (m/s) |
| angular | [double](#double) |  | (rad/s) |






<a name="rb-api-SE3Pose"></a>

### SE3Pose



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| position | [Vec3](#rb-api-Vec3) |  | (m) |
| quaternion | [Quaternion](#rb-api-Quaternion) |  |  |
| euler | [EulerAngleZYX](#rb-api-EulerAngleZYX) |  |  |






<a name="rb-api-Vec2"></a>

### Vec2



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| x | [double](#double) |  |  |
| y | [double](#double) |  |  |






<a name="rb-api-Vec3"></a>

### Vec3



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| x | [double](#double) |  |  |
| y | [double](#double) |  |  |
| z | [double](#double) |  |  |





 

 

 

 



<a name="rb_api_gripper_command-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/gripper_command.proto



<a name="rb-api-GripperInitializationRequest"></a>

### GripperInitializationRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  | Request header |
| name | [string](#string) |  |  |






<a name="rb-api-GripperInitializationResponse"></a>

### GripperInitializationResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  | Response header |






<a name="rb-api-GripperMoveRequest"></a>

### GripperMoveRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  | Request header |
| name | [string](#string) |  |  |
| position | [int32](#int32) |  |  |
| velocity | [int32](#int32) |  |  |
| force | [int32](#int32) |  |  |






<a name="rb-api-GripperMoveResponse"></a>

### GripperMoveResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  | Response header |





 

 

 

 



<a name="rb_api_gripper_command_service-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/gripper_command_service.proto


 

 

 


<a name="rb-api-GripperCommandService"></a>

### GripperCommandService


| Method Name | Request Type | Response Type | Description |
| ----------- | ------------ | ------------- | ------------|
| GripperInitialization | [GripperInitializationRequest](#rb-api-GripperInitializationRequest) | [GripperInitializationResponse](#rb-api-GripperInitializationResponse) |  |
| GripperMove | [GripperMoveRequest](#rb-api-GripperMoveRequest) | [GripperMoveResponse](#rb-api-GripperMoveResponse) | Joint command |

 



<a name="rb_api_head_command-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/head_command.proto



<a name="rb-api-HeadCommand"></a>

### HeadCommand







<a name="rb-api-HeadCommand-Feedback"></a>

### HeadCommand.Feedback



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header_feedback | [CommandHeader.Feedback](#rb-api-CommandHeader-Feedback) |  |  |
| joint_position_command_feedback | [JointPositionCommand.Feedback](#rb-api-JointPositionCommand-Feedback) |  |  |






<a name="rb-api-HeadCommand-Request"></a>

### HeadCommand.Request



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header | [CommandHeader.Request](#rb-api-CommandHeader-Request) |  |  |
| joint_position_command | [JointPositionCommand.Request](#rb-api-JointPositionCommand-Request) |  |  |





 

 

 

 



<a name="rb_api_header-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/header.proto



<a name="rb-api-CommonError"></a>

### CommonError



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| code | [CommonError.Code](#rb-api-CommonError-Code) |  | Error code |
| message | [string](#string) |  | Human-readable error message |






<a name="rb-api-RequestHeader"></a>

### RequestHeader
Standard request header


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_timestamp | [google.protobuf.Timestamp](#google-protobuf-Timestamp) |  | Client local system clock |






<a name="rb-api-ResponseHeader"></a>

### ResponseHeader
Standard response header


| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  | Echo |
| request_received_timestamp | [google.protobuf.Timestamp](#google-protobuf-Timestamp) |  | Robot clock |
| response_timestamp | [google.protobuf.Timestamp](#google-protobuf-Timestamp) |  | Robot clock |
| error | [CommonError](#rb-api-CommonError) |  | If set, there is error |





 


<a name="rb-api-CommonError-Code"></a>

### CommonError.Code


| Name | Number | Description |
| ---- | ------ | ----------- |
| CODE_UNSPECIFIED | 0 | Code is not specified. |
| CODE_OK | 1 | Not an error. Request was successful. |
| CODE_INTERNAL_SERVER_ERROR | 2 | Service experienced an unexpected error state. |
| CODE_INVALID_REQUEST | 3 | Ill-formed request. Request arguments were not valid. |


 

 

 



<a name="rb_api_log-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/log.proto



<a name="rb-api-GetLastLogRequest"></a>

### GetLastLogRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  |  |
| log_count | [int32](#int32) |  |  |






<a name="rb-api-GetLastLogResponse"></a>

### GetLastLogResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  |  |
| logs | [Log](#rb-api-Log) | repeated |  |






<a name="rb-api-GetLogStreamRequest"></a>

### GetLogStreamRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  |  |
| update_rate | [double](#double) |  | Hz |






<a name="rb-api-GetLogStreamResponse"></a>

### GetLogStreamResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  |  |
| logs | [Log](#rb-api-Log) | repeated |  |






<a name="rb-api-Log"></a>

### Log



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| timestamp | [google.protobuf.Timestamp](#google-protobuf-Timestamp) |  |  |
| robot_system_timestamp | [google.protobuf.Timestamp](#google-protobuf-Timestamp) |  |  |
| level | [Log.Level](#rb-api-Log-Level) |  |  |
| message | [string](#string) |  |  |






<a name="rb-api-SetLogLevelRequest"></a>

### SetLogLevelRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  |  |
| level | [Log.Level](#rb-api-Log-Level) |  |  |






<a name="rb-api-SetLogLevelResponse"></a>

### SetLogLevelResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  |  |





 


<a name="rb-api-Log-Level"></a>

### Log.Level


| Name | Number | Description |
| ---- | ------ | ----------- |
| LEVEL_TRACE | 0 |  |
| LEVEL_DEBUG | 1 |  |
| LEVEL_INFO | 2 |  |
| LEVEL_WARN | 3 |  |
| LEVEL_ERROR | 4 |  |
| LEVEL_CRITICAL | 5 |  |


 

 

 



<a name="rb_api_log_service-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/log_service.proto


 

 

 


<a name="rb-api-LogService"></a>

### LogService


| Method Name | Request Type | Response Type | Description |
| ----------- | ------------ | ------------- | ------------|
| GetLastLog | [GetLastLogRequest](#rb-api-GetLastLogRequest) | [GetLastLogResponse](#rb-api-GetLastLogResponse) |  |
| GetLogStream | [GetLogStreamRequest](#rb-api-GetLogStreamRequest) | [GetLogStreamResponse](#rb-api-GetLogStreamResponse) stream |  |
| SetLogLevel | [SetLogLevelRequest](#rb-api-SetLogLevelRequest) | [SetLogLevelResponse](#rb-api-SetLogLevelResponse) |  |

 



<a name="rb_api_mobility_command-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/mobility_command.proto



<a name="rb-api-MobilityCommand"></a>

### MobilityCommand







<a name="rb-api-MobilityCommand-Feedback"></a>

### MobilityCommand.Feedback



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header_feedback | [CommandHeader.Feedback](#rb-api-CommandHeader-Feedback) |  |  |
| joint_velocity_command_feedback | [JointVelocityCommand.Feedback](#rb-api-JointVelocityCommand-Feedback) |  |  |
| se2_velocity_command_feedback | [SE2VelocityCommand.Feedback](#rb-api-SE2VelocityCommand-Feedback) |  |  |






<a name="rb-api-MobilityCommand-Request"></a>

### MobilityCommand.Request



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header | [CommandHeader.Request](#rb-api-CommandHeader-Request) |  |  |
| joint_velocity_command | [JointVelocityCommand.Request](#rb-api-JointVelocityCommand-Request) |  |  |
| se2_velocity_command | [SE2VelocityCommand.Request](#rb-api-SE2VelocityCommand-Request) |  |  |





 

 

 

 



<a name="rb_api_parameter-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/parameter.proto



<a name="rb-api-GetParameterListRequest"></a>

### GetParameterListRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  |  |






<a name="rb-api-GetParameterListResponse"></a>

### GetParameterListResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  |  |
| parameters | [GetParameterListResponse.ParameterType](#rb-api-GetParameterListResponse-ParameterType) | repeated |  |






<a name="rb-api-GetParameterListResponse-ParameterType"></a>

### GetParameterListResponse.ParameterType



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| name | [string](#string) |  |  |
| type | [int32](#int32) |  | Type of parameter 0: int 1: double 2: std::string 3: std::array&lt;double, 3&gt; 4: std::array&lt;double, 6&gt; 5: std::array&lt;double, 7&gt; |






<a name="rb-api-GetParameterRequest"></a>

### GetParameterRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  |  |
| name | [string](#string) |  |  |






<a name="rb-api-GetParameterResponse"></a>

### GetParameterResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  |  |
| parameter | [string](#string) |  |  |






<a name="rb-api-ResetAllParametersToDefaultRequest"></a>

### ResetAllParametersToDefaultRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  |  |






<a name="rb-api-ResetAllParametersToDefaultResponse"></a>

### ResetAllParametersToDefaultResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  |  |






<a name="rb-api-ResetParameterToDefaultRequest"></a>

### ResetParameterToDefaultRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  |  |
| name | [string](#string) |  |  |






<a name="rb-api-ResetParameterToDefaultResponse"></a>

### ResetParameterToDefaultResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  |  |






<a name="rb-api-SetParameterRequest"></a>

### SetParameterRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  |  |
| name | [string](#string) |  |  |
| parameter | [string](#string) |  |  |






<a name="rb-api-SetParameterResponse"></a>

### SetParameterResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  |  |





 

 

 

 



<a name="rb_api_parameter_service-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/parameter_service.proto


 

 

 


<a name="rb-api-ParameterService"></a>

### ParameterService


| Method Name | Request Type | Response Type | Description |
| ----------- | ------------ | ------------- | ------------|
| ResetAllParametersToDefault | [ResetAllParametersToDefaultRequest](#rb-api-ResetAllParametersToDefaultRequest) | [ResetAllParametersToDefaultResponse](#rb-api-ResetAllParametersToDefaultResponse) |  |
| ResetParameterToDefault | [ResetParameterToDefaultRequest](#rb-api-ResetParameterToDefaultRequest) | [ResetParameterToDefaultResponse](#rb-api-ResetParameterToDefaultResponse) |  |
| GetParameter | [GetParameterRequest](#rb-api-GetParameterRequest) | [GetParameterResponse](#rb-api-GetParameterResponse) |  |
| SetParameter | [SetParameterRequest](#rb-api-SetParameterRequest) | [SetParameterResponse](#rb-api-SetParameterResponse) |  |
| GetParameterList | [GetParameterListRequest](#rb-api-GetParameterListRequest) | [GetParameterListResponse](#rb-api-GetParameterListResponse) |  |

 



<a name="rb_api_ping-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/ping.proto



<a name="rb-api-PingRequest"></a>

### PingRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  |  |






<a name="rb-api-PingResponse"></a>

### PingResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  |  |





 

 

 

 



<a name="rb_api_ping_service-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/ping_service.proto


 

 

 


<a name="rb-api-PingService"></a>

### PingService


| Method Name | Request Type | Response Type | Description |
| ----------- | ------------ | ------------- | ------------|
| Ping | [PingRequest](#rb-api-PingRequest) | [PingResponse](#rb-api-PingResponse) |  |

 



<a name="rb_api_power-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/power.proto



<a name="rb-api-JointCommandRequest"></a>

### JointCommandRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  | Request header |
| name | [string](#string) |  | Motor ID |
| command | [JointCommandRequest.Command](#rb-api-JointCommandRequest-Command) |  |  |






<a name="rb-api-JointCommandResponse"></a>

### JointCommandResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  | Response header |
| status | [JointCommandResponse.Status](#rb-api-JointCommandResponse-Status) |  |  |
| message | [string](#string) |  | Human-readable message for status |






<a name="rb-api-PowerCommandRequest"></a>

### PowerCommandRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  | Request header |
| name | [string](#string) |  | Power ID |
| command | [PowerCommandRequest.Command](#rb-api-PowerCommandRequest-Command) |  |  |






<a name="rb-api-PowerCommandResponse"></a>

### PowerCommandResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  | Response header |
| status | [PowerCommandResponse.Status](#rb-api-PowerCommandResponse-Status) |  |  |
| message | [string](#string) |  | Human-readable message for status |






<a name="rb-api-ToolFlangePowerCommandRequest"></a>

### ToolFlangePowerCommandRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  | Request header |
| name | [string](#string) |  | Tool Flange name |
| command | [ToolFlangePowerCommandRequest.Command](#rb-api-ToolFlangePowerCommandRequest-Command) |  |  |






<a name="rb-api-ToolFlangePowerCommandResponse"></a>

### ToolFlangePowerCommandResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  | Response header |





 


<a name="rb-api-JointCommandRequest-Command"></a>

### JointCommandRequest.Command
Modes for joint/motor command

| Name | Number | Description |
| ---- | ------ | ----------- |
| COMMAND_UNKNOWN | 0 |  |
| COMMAND_SERVO_ON | 1 |  |
| COMMAND_BRAKE_ENGAGE | 2 |  |
| COMMAND_BRAKE_RELEASE | 3 |  |
| COMMAND_HOME_OFFSET_RST | 4 |  |



<a name="rb-api-JointCommandResponse-Status"></a>

### JointCommandResponse.Status


| Name | Number | Description |
| ---- | ------ | ----------- |
| STATUS_UNKNOWN | 0 |  |
| STATUS_SUCCESS | 1 |  |
| STATUS_INTERNAL_ERROR | 2 |  |



<a name="rb-api-PowerCommandRequest-Command"></a>

### PowerCommandRequest.Command
Power command

| Name | Number | Description |
| ---- | ------ | ----------- |
| COMMAND_UNKNOWN | 0 |  |
| COMMAND_POWER_ON | 1 |  |
| COMMAND_POWER_OFF | 2 |  |



<a name="rb-api-PowerCommandResponse-Status"></a>

### PowerCommandResponse.Status


| Name | Number | Description |
| ---- | ------ | ----------- |
| STATUS_UNKNOWN | 0 |  |
| STATUS_SUCCESS | 1 |  |
| STATUS_INTERNAL_ERROR | 2 |  |



<a name="rb-api-ToolFlangePowerCommandRequest-Command"></a>

### ToolFlangePowerCommandRequest.Command


| Name | Number | Description |
| ---- | ------ | ----------- |
| COMMAND_UNKNOWN | 0 |  |
| COMMAND_POWER_OFF | 1 |  |
| COMMAND_POWER_12V | 2 |  |
| COMMAND_POWER_24V | 3 |  |


 

 

 



<a name="rb_api_power_service-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/power_service.proto


 

 

 


<a name="rb-api-PowerService"></a>

### PowerService


| Method Name | Request Type | Response Type | Description |
| ----------- | ------------ | ------------- | ------------|
| PowerCommand | [PowerCommandRequest](#rb-api-PowerCommandRequest) | [PowerCommandResponse](#rb-api-PowerCommandResponse) | Control power of the robot |
| JointCommand | [JointCommandRequest](#rb-api-JointCommandRequest) | [JointCommandResponse](#rb-api-JointCommandResponse) | Joint command |
| ToolFlangePowerCommand | [ToolFlangePowerCommandRequest](#rb-api-ToolFlangePowerCommandRequest) | [ToolFlangePowerCommandResponse](#rb-api-ToolFlangePowerCommandResponse) | Tool Flange |

 



<a name="rb_api_robot_command-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/robot_command.proto



<a name="rb-api-RobotCommand"></a>

### RobotCommand







<a name="rb-api-RobotCommand-Feedback"></a>

### RobotCommand.Feedback



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header_feedback | [CommandHeader.Feedback](#rb-api-CommandHeader-Feedback) |  |  |
| whole_body_command_feedback | [WholeBodyCommand.Feedback](#rb-api-WholeBodyCommand-Feedback) |  |  |
| component_based_command_feedback | [ComponentBasedCommand.Feedback](#rb-api-ComponentBasedCommand-Feedback) |  |  |
| jog_command_feedback | [JogCommand.Feedback](#rb-api-JogCommand-Feedback) |  |  |
| status | [RobotCommand.Feedback.Status](#rb-api-RobotCommand-Feedback-Status) |  |  |
| finish_code | [RobotCommand.Feedback.FinishCode](#rb-api-RobotCommand-Feedback-FinishCode) |  |  |






<a name="rb-api-RobotCommand-Request"></a>

### RobotCommand.Request



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header | [CommandHeader.Request](#rb-api-CommandHeader-Request) |  |  |
| whole_body_command | [WholeBodyCommand.Request](#rb-api-WholeBodyCommand-Request) |  |  |
| component_based_command | [ComponentBasedCommand.Request](#rb-api-ComponentBasedCommand-Request) |  |  |
| jog_command | [JogCommand.Request](#rb-api-JogCommand-Request) |  |  |






<a name="rb-api-RobotCommandRequest"></a>

### RobotCommandRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  |  |
| robot_command | [RobotCommand.Request](#rb-api-RobotCommand-Request) |  |  |
| priority | [int32](#int32) |  |  |






<a name="rb-api-RobotCommandResponse"></a>

### RobotCommandResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  |  |
| feedback | [RobotCommand.Feedback](#rb-api-RobotCommand-Feedback) |  |  |





 


<a name="rb-api-RobotCommand-Feedback-FinishCode"></a>

### RobotCommand.Feedback.FinishCode


| Name | Number | Description |
| ---- | ------ | ----------- |
| FINISH_CODE_UNKNOWN | 0 |  |
| FINISH_CODE_OK | 1 |  |
| FINISH_CODE_CANCELED | 2 |  |
| FINISH_CODE_PREEMPTED | 3 |  |
| FINISH_CODE_INITIALIZED_FAILED | 4 |  |
| FINISH_CODE_CONTROL_MANAGER_IDLE | 5 |  |
| FINISH_CODE_CONTROL_MANAGER_FAULT | 6 |  |
| FINISH_CODE_UNEXPECTED_STATE | 7 |  |



<a name="rb-api-RobotCommand-Feedback-Status"></a>

### RobotCommand.Feedback.Status


| Name | Number | Description |
| ---- | ------ | ----------- |
| STATUS_IDLE | 0 |  |
| STATUS_INITIALIZING | 1 |  |
| STATUS_RUNNING | 2 |  |
| STATUS_FINISHED | 3 |  |


 

 

 



<a name="rb_api_robot_command_service-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/robot_command_service.proto


 

 

 


<a name="rb-api-RobotCommandService"></a>

### RobotCommandService


| Method Name | Request Type | Response Type | Description |
| ----------- | ------------ | ------------- | ------------|
| RobotCommand | [RobotCommandRequest](#rb-api-RobotCommandRequest) | [RobotCommandResponse](#rb-api-RobotCommandResponse) | In case of sending and receiving a single command |
| RobotCommandStream | [RobotCommandRequest](#rb-api-RobotCommandRequest) stream | [RobotCommandResponse](#rb-api-RobotCommandResponse) stream | In case of sending and receiving commands continuously |

 



<a name="rb_api_robot_info-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/robot_info.proto



<a name="rb-api-BatteryInfo"></a>

### BatteryInfo







<a name="rb-api-EMOInfo"></a>

### EMOInfo



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| name | [string](#string) |  |  |






<a name="rb-api-GetRobotInfoRequest"></a>

### GetRobotInfoRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  |  |






<a name="rb-api-GetRobotInfoResponse"></a>

### GetRobotInfoResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  |  |
| robot_info | [RobotInfo](#rb-api-RobotInfo) |  |  |






<a name="rb-api-GetRobotModelRequest"></a>

### GetRobotModelRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  |  |






<a name="rb-api-GetRobotModelResponse"></a>

### GetRobotModelResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  |  |
| model | [string](#string) |  |  |






<a name="rb-api-ImportRobotModelRequest"></a>

### ImportRobotModelRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  |  |
| name | [string](#string) |  |  |
| model | [string](#string) |  |  |






<a name="rb-api-ImportRobotModelResponse"></a>

### ImportRobotModelResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  |  |






<a name="rb-api-JointInfo"></a>

### JointInfo



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| name | [string](#string) |  |  |
| has_brake | [bool](#bool) |  |  |






<a name="rb-api-PowerInfo"></a>

### PowerInfo



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| name | [string](#string) |  |  |






<a name="rb-api-RobotInfo"></a>

### RobotInfo



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| robot_version | [string](#string) |  |  |
| sdk_commit_id | [string](#string) |  |  |
| battery_info | [BatteryInfo](#rb-api-BatteryInfo) |  |  |
| power_infos | [PowerInfo](#rb-api-PowerInfo) | repeated |  |
| emo_infos | [EMOInfo](#rb-api-EMOInfo) | repeated |  |
| degree_of_freedom | [int32](#int32) |  |  |
| joint_infos | [JointInfo](#rb-api-JointInfo) | repeated |  |
| mobility_joint_idx | [uint32](#uint32) | repeated |  |
| body_joint_idx | [uint32](#uint32) | repeated |  |
| head_joint_idx | [uint32](#uint32) | repeated |  |





 

 

 

 



<a name="rb_api_robot_info_service-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/robot_info_service.proto


 

 

 


<a name="rb-api-RobotInfoService"></a>

### RobotInfoService


| Method Name | Request Type | Response Type | Description |
| ----------- | ------------ | ------------- | ------------|
| GetRobotInfo | [GetRobotInfoRequest](#rb-api-GetRobotInfoRequest) | [GetRobotInfoResponse](#rb-api-GetRobotInfoResponse) |  |
| GetRobotModel | [GetRobotModelRequest](#rb-api-GetRobotModelRequest) | [GetRobotModelResponse](#rb-api-GetRobotModelResponse) |  |
| ImportRobotModel | [ImportRobotModelRequest](#rb-api-ImportRobotModelRequest) | [ImportRobotModelResponse](#rb-api-ImportRobotModelResponse) |  |

 



<a name="rb_api_robot_state-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/robot_state.proto



<a name="rb-api-BatteryState"></a>

### BatteryState



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| voltage | [double](#double) |  | V |
| current | [double](#double) |  | Amp |
| level_percent | [double](#double) |  | % |






<a name="rb-api-Collision"></a>

### Collision



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| link1 | [string](#string) |  |  |
| link2 | [string](#string) |  |  |
| position1 | [Vec3](#rb-api-Vec3) |  |  |
| position2 | [Vec3](#rb-api-Vec3) |  |  |
| distance | [double](#double) |  |  |






<a name="rb-api-EMOState"></a>

### EMOState



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| state | [EMOState.State](#rb-api-EMOState-State) |  |  |






<a name="rb-api-FTSensorData"></a>

### FTSensorData



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| time_since_last_update | [google.protobuf.Duration](#google-protobuf-Duration) |  |  |
| force | [Vec3](#rb-api-Vec3) |  |  |
| torque | [Vec3](#rb-api-Vec3) |  |  |






<a name="rb-api-GetControlManagerStateRequest"></a>

### GetControlManagerStateRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  |  |






<a name="rb-api-GetControlManagerStateResponse"></a>

### GetControlManagerStateResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  |  |
| control_manager_state | [ControlManagerState](#rb-api-ControlManagerState) |  |  |






<a name="rb-api-GetRobotStateRequest"></a>

### GetRobotStateRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  |  |






<a name="rb-api-GetRobotStateResponse"></a>

### GetRobotStateResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  |  |
| robot_state | [RobotState](#rb-api-RobotState) |  |  |
| control_manager_state | [ControlManagerState](#rb-api-ControlManagerState) |  |  |






<a name="rb-api-GetRobotStateStreamRequest"></a>

### GetRobotStateStreamRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  |  |
| update_rate | [double](#double) |  | Hz |






<a name="rb-api-GetRobotStateStreamResponse"></a>

### GetRobotStateStreamResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  |  |
| robot_state | [RobotState](#rb-api-RobotState) |  |  |
| control_manager_state | [ControlManagerState](#rb-api-ControlManagerState) |  |  |






<a name="rb-api-JointState"></a>

### JointState



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| is_ready | [bool](#bool) |  |  |
| fet_state | [JointState.FETState](#rb-api-JointState-FETState) |  |  |
| run_state | [JointState.RunState](#rb-api-JointState-RunState) |  |  |
| init_state | [JointState.InitializationState](#rb-api-JointState-InitializationState) |  |  |
| motor_type | [uint32](#uint32) |  | MOTOR STATE |
| motor_state | [uint64](#uint64) |  |  |
| time_since_last_update | [google.protobuf.Duration](#google-protobuf-Duration) |  |  |
| power_on | [bool](#bool) |  |  |
| position | [double](#double) |  |  |
| velocity | [double](#double) |  |  |
| current | [double](#double) |  |  |
| torque | [double](#double) |  |  |
| target_position | [double](#double) |  |  |
| target_velocity | [double](#double) |  |  |
| target_feedback_gain | [uint32](#uint32) |  |  |
| target_feedforward_torque | [double](#double) |  |  |






<a name="rb-api-PowerState"></a>

### PowerState



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| state | [PowerState.State](#rb-api-PowerState-State) |  |  |
| voltage | [double](#double) |  |  |






<a name="rb-api-ResetOdometryRequest"></a>

### ResetOdometryRequest



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| request_header | [RequestHeader](#rb-api-RequestHeader) |  |  |
| initial_pose | [SE2Pose](#rb-api-SE2Pose) |  |  |






<a name="rb-api-ResetOdometryResponse"></a>

### ResetOdometryResponse



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| response_header | [ResponseHeader](#rb-api-ResponseHeader) |  |  |






<a name="rb-api-RobotState"></a>

### RobotState



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| timestamp | [google.protobuf.Timestamp](#google-protobuf-Timestamp) |  |  |
| system_stat | [SystemStat](#rb-api-SystemStat) |  | System Statistic |
| battery_state | [BatteryState](#rb-api-BatteryState) |  | Battery State |
| power_states | [PowerState](#rb-api-PowerState) | repeated | Power State |
| emo_states | [EMOState](#rb-api-EMOState) | repeated | EMO state |
| joint_states | [JointState](#rb-api-JointState) | repeated | Joint State |
| tool_flange_right | [ToolFlangeState](#rb-api-ToolFlangeState) |  | Tool Flange State |
| tool_flange_left | [ToolFlangeState](#rb-api-ToolFlangeState) |  |  |
| ft_sensor_right | [FTSensorData](#rb-api-FTSensorData) |  | Force Torque Sensor |
| ft_sensor_left | [FTSensorData](#rb-api-FTSensorData) |  |  |
| is_ready | [double](#double) | repeated |  |
| position | [double](#double) | repeated |  |
| velocity | [double](#double) | repeated |  |
| current | [double](#double) | repeated |  |
| torque | [double](#double) | repeated |  |
| target_position | [double](#double) | repeated |  |
| target_velocity | [double](#double) | repeated |  |
| target_feedback_gain | [uint32](#uint32) | repeated |  |
| target_feedforward_torque | [double](#double) | repeated |  |
| odometry | [SE2Pose](#rb-api-SE2Pose) |  | Mobility State |
| center_of_mass | [Vec3](#rb-api-Vec3) |  | Center Of Mass

Position of center of mass with respect t base link |
| collisions | [Collision](#rb-api-Collision) | repeated | Collisions |






<a name="rb-api-SystemStat"></a>

### SystemStat



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| cpu_usage | [double](#double) |  | % |
| memory_usage | [double](#double) |  | % |
| uptime | [double](#double) |  | sec |
| program_uptime | [double](#double) |  | sec |






<a name="rb-api-ToolFlangeState"></a>

### ToolFlangeState



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| time_since_last_update | [google.protobuf.Duration](#google-protobuf-Duration) |  |  |
| gyro | [Vec3](#rb-api-Vec3) |  |  |
| acceleration | [Vec3](#rb-api-Vec3) |  |  |
| switch_A | [bool](#bool) |  |  |
| output_voltage | [int32](#int32) |  |  |





 


<a name="rb-api-EMOState-State"></a>

### EMOState.State


| Name | Number | Description |
| ---- | ------ | ----------- |
| STATE_RELEASED | 0 |  |
| STATE_PRESSED | 1 |  |



<a name="rb-api-JointState-FETState"></a>

### JointState.FETState


| Name | Number | Description |
| ---- | ------ | ----------- |
| FET_STATE_UNKNOWN | 0 |  |
| FET_STATE_ON | 1 |  |
| FET_STATE_OFF | 2 |  |



<a name="rb-api-JointState-InitializationState"></a>

### JointState.InitializationState


| Name | Number | Description |
| ---- | ------ | ----------- |
| INIT_STATE_UNKNOWN | 0 |  |
| INIT_STATE_INITIALIZED | 1 |  |
| INIT_STATE_UNINITIALIZED | 2 |  |



<a name="rb-api-JointState-RunState"></a>

### JointState.RunState


| Name | Number | Description |
| ---- | ------ | ----------- |
| RUN_STATE_UNKNOWN | 0 |  |
| RUN_STATE_CONTROL_ON | 1 |  |
| RUN_STATE_CONTROL_OFF | 2 |  |



<a name="rb-api-PowerState-State"></a>

### PowerState.State


| Name | Number | Description |
| ---- | ------ | ----------- |
| STATE_UNKNOWN | 0 |  |
| STATE_POWER_ON | 1 |  |
| STATE_POWER_OFF | 2 |  |


 

 

 



<a name="rb_api_robot_state_service-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/robot_state_service.proto


 

 

 


<a name="rb-api-RobotStateService"></a>

### RobotStateService


| Method Name | Request Type | Response Type | Description |
| ----------- | ------------ | ------------- | ------------|
| GetRobotState | [GetRobotStateRequest](#rb-api-GetRobotStateRequest) | [GetRobotStateResponse](#rb-api-GetRobotStateResponse) |  |
| GetRobotStateStream | [GetRobotStateStreamRequest](#rb-api-GetRobotStateStreamRequest) | [GetRobotStateStreamResponse](#rb-api-GetRobotStateStreamResponse) stream |  |
| GetControlManagerState | [GetControlManagerStateRequest](#rb-api-GetControlManagerStateRequest) | [GetControlManagerStateResponse](#rb-api-GetControlManagerStateResponse) |  |
| ResetOdometry | [ResetOdometryRequest](#rb-api-ResetOdometryRequest) | [ResetOdometryResponse](#rb-api-ResetOdometryResponse) |  |

 



<a name="rb_api_torso_command-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/torso_command.proto



<a name="rb-api-TorsoCommand"></a>

### TorsoCommand







<a name="rb-api-TorsoCommand-Feedback"></a>

### TorsoCommand.Feedback



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header_feedback | [CommandHeader.Feedback](#rb-api-CommandHeader-Feedback) |  |  |
| joint_position_command_feedback | [JointPositionCommand.Feedback](#rb-api-JointPositionCommand-Feedback) |  |  |
| gravity_compensation_command_feedback | [GravityCompensationCommand.Feedback](#rb-api-GravityCompensationCommand-Feedback) |  |  |
| cartesian_command_feedback | [CartesianCommand.Feedback](#rb-api-CartesianCommand-Feedback) |  |  |
| impedance_control_command_feedback | [ImpedanceControlCommand.Feedback](#rb-api-ImpedanceControlCommand-Feedback) |  |  |
| optimal_control_command_feedback | [OptimalControlCommand.Feedback](#rb-api-OptimalControlCommand-Feedback) |  |  |






<a name="rb-api-TorsoCommand-Request"></a>

### TorsoCommand.Request



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header | [CommandHeader.Request](#rb-api-CommandHeader-Request) |  |  |
| joint_position_command | [JointPositionCommand.Request](#rb-api-JointPositionCommand-Request) |  |  |
| gravity_compensation_command | [GravityCompensationCommand.Request](#rb-api-GravityCompensationCommand-Request) |  |  |
| cartesian_command | [CartesianCommand.Request](#rb-api-CartesianCommand-Request) |  |  |
| impedance_control_command | [ImpedanceControlCommand.Request](#rb-api-ImpedanceControlCommand-Request) |  |  |
| optimal_control_command | [OptimalControlCommand.Request](#rb-api-OptimalControlCommand-Request) |  |  |





 

 

 

 



<a name="rb_api_whole_body_command-proto"></a>
<p align="right"><a href="#top">Top</a></p>

## rb/api/whole_body_command.proto



<a name="rb-api-WholeBodyCommand"></a>

### WholeBodyCommand







<a name="rb-api-WholeBodyCommand-Feedback"></a>

### WholeBodyCommand.Feedback



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header_feedback | [CommandHeader.Feedback](#rb-api-CommandHeader-Feedback) |  |  |
| stop_command_feedback | [StopCommand.Feedback](#rb-api-StopCommand-Feedback) |  |  |
| real_time_control_command_feedback | [RealTimeControlCommand.Feedback](#rb-api-RealTimeControlCommand-Feedback) |  |  |






<a name="rb-api-WholeBodyCommand-Request"></a>

### WholeBodyCommand.Request



| Field | Type | Label | Description |
| ----- | ---- | ----- | ----------- |
| command_header | [CommandHeader.Request](#rb-api-CommandHeader-Request) |  |  |
| stop_command | [StopCommand.Request](#rb-api-StopCommand-Request) |  |  |
| real_time_control_command | [RealTimeControlCommand.Request](#rb-api-RealTimeControlCommand-Request) |  |  |





 

 

 

 



## Scalar Value Types

| .proto Type | Notes | C++ | Java | Python | Go | C# | PHP | Ruby |
| ----------- | ----- | --- | ---- | ------ | -- | -- | --- | ---- |
| <a name="double" /> double |  | double | double | float | float64 | double | float | Float |
| <a name="float" /> float |  | float | float | float | float32 | float | float | Float |
| <a name="int32" /> int32 | Uses variable-length encoding. Inefficient for encoding negative numbers – if your field is likely to have negative values, use sint32 instead. | int32 | int | int | int32 | int | integer | Bignum or Fixnum (as required) |
| <a name="int64" /> int64 | Uses variable-length encoding. Inefficient for encoding negative numbers – if your field is likely to have negative values, use sint64 instead. | int64 | long | int/long | int64 | long | integer/string | Bignum |
| <a name="uint32" /> uint32 | Uses variable-length encoding. | uint32 | int | int/long | uint32 | uint | integer | Bignum or Fixnum (as required) |
| <a name="uint64" /> uint64 | Uses variable-length encoding. | uint64 | long | int/long | uint64 | ulong | integer/string | Bignum or Fixnum (as required) |
| <a name="sint32" /> sint32 | Uses variable-length encoding. Signed int value. These more efficiently encode negative numbers than regular int32s. | int32 | int | int | int32 | int | integer | Bignum or Fixnum (as required) |
| <a name="sint64" /> sint64 | Uses variable-length encoding. Signed int value. These more efficiently encode negative numbers than regular int64s. | int64 | long | int/long | int64 | long | integer/string | Bignum |
| <a name="fixed32" /> fixed32 | Always four bytes. More efficient than uint32 if values are often greater than 2^28. | uint32 | int | int | uint32 | uint | integer | Bignum or Fixnum (as required) |
| <a name="fixed64" /> fixed64 | Always eight bytes. More efficient than uint64 if values are often greater than 2^56. | uint64 | long | int/long | uint64 | ulong | integer/string | Bignum |
| <a name="sfixed32" /> sfixed32 | Always four bytes. | int32 | int | int | int32 | int | integer | Bignum or Fixnum (as required) |
| <a name="sfixed64" /> sfixed64 | Always eight bytes. | int64 | long | int/long | int64 | long | integer/string | Bignum |
| <a name="bool" /> bool |  | bool | boolean | boolean | bool | bool | boolean | TrueClass/FalseClass |
| <a name="string" /> string | A string must always contain UTF-8 encoded or 7-bit ASCII text. | string | String | str/unicode | string | string | string | String (UTF-8) |
| <a name="bytes" /> bytes | May contain any arbitrary sequence of bytes. | string | ByteString | str | []byte | ByteString | string | String (ASCII-8BIT) |

