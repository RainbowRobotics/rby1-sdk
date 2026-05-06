add_subdirectory(dynamics)

set(EXAMPLES
        # Numbered examples (matching Python numbering)
        01_hello_rby1
        02_robot_model_setting
        03_robot_state
        04_parameter
        05_robot_state_stream
        06_state_tool_flange
        07_power
        08_check_firmware_version
        09_get_pid_gain
        10_set_pid_gain
        11_factory_default_pid_gain
        12_led
        13_set_system_time
        14_wifi
        15_log
        16_log_stream
        17_fault_log
        18_dynamics_modeling
        19_dynamics_robot
        20_leader_arm_state_check
        21_record
        22_replay
        23_zero_pose
        24_demo_motion
        25_cancel_control
        26_collisions
        27_cartesian_impedance_control
        28_joint_impedance_control
        29_joint_group_command
        30_multi_controls
        31_command_stream
        32_cartesian_command_stream
        33_real_time_control
        34_leader_arm_teleop_with_monitor
        35_brake_test
        36_mobile_test
        37_serial_device
        38_serial_communication
)

foreach (SRC IN LISTS EXAMPLES)
    set(EXECUTABLE_NAME example_${SRC})
    add_executable(${EXECUTABLE_NAME}
            ${SRC}.cpp
    )
    target_link_libraries(${EXECUTABLE_NAME} PRIVATE
            rby1-sdk
        
            DynamixelSDK
            OsqpEigen::OsqpEigen
            tinyxml2::tinyxml2
    )
    target_compile_definitions(${EXECUTABLE_NAME} PUBLIC
            -DPATH="${CMAKE_CURRENT_SOURCE_DIR}"
            -DMODELS_PATH="${CMAKE_SOURCE_DIR}/models/"
    )
    message(STATUS ${EXECUTABLE_NAME})
endforeach ()
