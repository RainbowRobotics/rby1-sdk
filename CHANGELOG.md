## v0.8.3 (2025-09-02)

### 🐛 Fixes

- tap conflict on github action
- add target position in master arm state
- reinitialize variable when restarting master arm

## v0.8.2 (2025-08-18)

### ♻️ Refactor

- **example**: adjust torque limit in current-based position control

### 🐛 Fixes

- remove exit when master arm initialization is failed
- use traget torque in current based position control mode
- skip pip upgrade to avoid Homebrew uninstall error on macOS
- model a v1.2 collisions for last links / model m mobile base kinematics param

## v0.8.1 (2025-07-15)

### 🐛 Fixes

- add python wrapping class for JointImpedanceControlCommandFeedback, JointGroupPositionCommandFeedback

## v0.8.0 (2025-07-15)

### ✨ Features

- add a v1.2 model
- add joint group position command
- add fault log listing and file download support
- add RBY1_SDK_API export macro for cross-platform symbol visibility
- add model A v1.1
- add digital i/o in proto
- set tool flange both digital outputs at the same time
- add preset position function for pvl sensor

### 🐛 Fixes

- rename enum kInitializedFailed to kInitializationFailed
- rename debug shared object
- add rpc/sdk version into robot info
- correct shoulder 0 link frame orientation in rby1a urdf model
- remove models from sdk c++ archive
- add /bigobj for MSVC to prevent section limit compilation error
- define M_PI
- add RBY1_SDK_API to QPSolver
- add chrono header
- remove RBY1_SDK_API in template classes
- **cmake**: make RBY1_SDK_STATIC a PUBLIC definition for static builds
- **cmake**: use RBY1_SDK_STATIC
- **cmake**: apply RBY1_SDK_EXPORTS only for shared library builds
- remove auto select apt mirror
- change duty to state for tf io

## v0.7.0 (2025-05-30)

### ♻️ Refactor

- serial header

### ✨ Features

- add python wrapper for serial comm
- add device list rpc
- add serial
- add standard upper body model
- add cartesian impedance control command

### 🐛 Fixes

- **build**: override CC env variable for libcap
- change wheel rotation direction on m model
- python wrapper string to bytes
- typo
- typo
- osqp

## v0.6.0 (2025-05-02)

### ♻️ Refactor

- **example**: update examples for get/set pid gains

### ✨ Features

- retrieve torque limit from urdf and support joint impedance control command in builder
- add proto for joint impedance control command
- **impedance_control**: introduce damping ratio for adjusting damping gain
- **robot_state**: add temperature in robot state

### 🐛 Fixes

- remove designated initializers
- change dynamixel interval time
- change m model userdata size
- typo
- **osqp,-osqp-eigen**: update version of osqp and osqp-eigen to v1.0.0 and v0.10.0, repectively
- remove ready_for_command from state

## v0.5.0 (2025-03-28)

### ✨ Features

- support wifi functions in python
- add wifi proto files
- add pos/orientation error in cartesian command feedback
- add error_scaling parameter for optimal control
- calculate manipulability
- support master arm class in python
- add wait_for_control_ready function
- add time_based_progress and position_based_progress feedback to joint position command (#83)

### 🐛 Fixes

- parameter change
- typo
- change qp solver exception (error) mechanism
- rename cartesian command builder parameter
- change parameter for demo motion
- resolve bugs in qc_gui and demo motion
- update qc_gui to support A, T5, and M models
- modify python teleoperation example to support M and T5 models
- update python example code for multi-model support (#84)
- adjust wrist yaw2 range of motion in model.urdf for A, T5, and M models (#85)
- typo
- set velocity_tracking_gain as zero
- restore gripper thread and refine gripper min/max detection in teleoperation example
- add teleoperation example using master arm class
- typo
- modify optimal control formulation with null space projection
- mark limits as reuiqred argument
- apply velocity limit
- change the way to calculate error in optimal control
- remove velocity tracking gain in optimal control command
- consider current velocity in optimal controller
- master arm class initialization failed
- fix link error on windows
- notebook example
- notebook example
- include chrono header
- use CancelControl() in Command and CommandStream handler
- rename parameters
- minor typo

## v0.4.1 (2025-02-21)

### 🐛 Fixes

- update pypi version to 0.4.1 due to versioning issue (skip 0.4.0)

## v0.4.0 (2025-02-21)

### ♻️ Refactor

- update bind_robot and bind_state in python
- **demo_motion**: rename waist to torso in demo_motion

### ✨ Features

- add example for checking firmware version
- Enhance JointInfo to include firmware version and product name
- add python teleoperation example (performance improvements needed)
- add m model (urdf, mjcf) and update motor index for m model
- support mecanum mobile base in dynamics lib
- add api for battery config (#42)
- add api for system time control
- add api to control the led (#40)

### 🐛 Fixes

- **robot.h**: add missing break in switch statement
- support model m in python
- bugfix in loader mecanum mobile base
- add ready for command
- **robot_state**: add gravity term in robot_state
- change Color (proto message) component type from float to uint32

## v0.3.0 (2024-12-20)

### ♻️ Refactor

- remove docs files in protos
- demo_motion
- **event_loop**: improve event_loop cyclic_task function
- restructure gain retrieval and return as struct in rby1-sdk
- clean up mobility_command example

### ✨ Features

- add CancelControl function to stop ongoing current control
- update demo motion minimum time and velocity limit scaling
- **parameter**: add support for temporary parameter setting in SetParameter
- add demo motion example for master arm operation
- **model.urdf**: update joint limit(torso_3)
- update joint limit
- update default gain values
- add grpc auto-generated code for web ui
- add motor temperature information
- expand gripper tip collision area
- update functionality for joint position target in cartesian command
- add joint position target to cartesian command (WIP)
- add overloaded constructor to PIDGain and add Python examples for gain set/get
- improve gain management and accessibility
- add head gain example and mutex lock for Dynamixel gain and motro state retrieval
- add examples for setting PID gains and factory reset
- add functionality to set and get pid gains
- add docker-compose.sim.yaml for ARM simulation setup
- add docker-compose.sim.yaml for simulation setup
- break eng/rel + home offset
- proto docs update
- add gripper test (python)
- gripper test for python
- update quick ui + qc ui
- qc gui servo on update
- qc gui update
- teleop + demo motion update + quick ui
- quick ui update(zero + ems)
- impedance update
- quick ui update
- record & replay example update

### 🐛 Fixes

- update gripper range detection
- update head current casting after reverting to previous implementation
- **dynamixel_bus**: improve motor state reader function
- make joint values realistic (#33)
- **robot_command_builder.cpp**: update pybind defaults and include stl header
- update current reading logic for dynamixel
- **demo_motion.cpp**: correct typo in demo motion setjointpositiontarget -> addjointpositiontarget
- check done before cancel call
- correct PID gain mapping and remove redundant torque disable call
- align mobility SE2 and joint velocity direction with RPC program for simulator consistency
- update network settings in docker-compose
- **master_arm**: fix out of bound access bug
- **model.urdf**: Change velocity limits for urdf torso_0, torso_1, and left_arm_5
- **dynamixel-bus**: fix bug
- change delay for dynamixel to 100us
- get control manager state in start_state_update cb function
- typo update
- typo
- update teleoperation example

## v0.2.0 (2024-10-02)

### ✨ Features

- support dynamics in python

## v0.1.11 (2024-09-30)

### 🐛 Fixes

- remove unnecessary dependency
- resolve package Python requirement issue

## v0.1.10 (2024-09-30)

### 🐛 Fixes

- make sdist once

## v0.1.9 (2024-09-30)

### 🐛 Fixes

- remove build folder before Poetry build

## v0.1.8 (2024-09-30)

### 🐛 Fixes

- run action on bash

## v0.1.7 (2024-09-30)

### 🐛 Fixes

- **release.yml**: update ci

## v0.1.6 (2024-09-30)

### 🐛 Fixes

- **release.yml**: update release action

## v0.1.5 (2024-09-30)

### 🐛 Fixes

- **release.yml**: fix

## v0.1.4 (2024-09-30)

### 🐛 Fixes

- **release.yml**: fix path on windows

## v0.1.3 (2024-09-30)

### 🐛 Fixes

- **release.yml**: use conditions for poetry install

## v0.1.2 (2024-09-30)

### 🐛 Fixes

- **release.yml**: change poetry path on windows

## v0.1.1 (2024-09-30)

### 🐛 Fixes

- fix ci

## v0.1.0 (2024-09-30)

### ✨ Features

- update python demo code
- update python demo code
- update self_collision & demo_motion
- proto docs test
- update demo motion2
- update module_test master_arm
- teleoperation example updat
- gripper test update
- update rpc address
- update demo motion
- update example
- update cmakelists
- urdf update
- change input dev name
- add sdk_commit_id (python binding)
- add get/set robot model from/to rpc
- add unlimited mode functionality
- add init protos

### 🐛 Fixes

- cast method
- macos test
- add build.py for build wheel
- fix typo
