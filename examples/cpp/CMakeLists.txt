add_subdirectory(dynamics)
add_subdirectory(module_test)
add_subdirectory(network)
set(EXAMPLES
        trapezoidal_motion_generator
        so3
        optimal_control
        velocity_estimation

        teleoperation_with_joint_mapping

        kinematic_calibration
        kinematic_calibration2

        log
        reset_odometry
        power_command

        get_robot_state
        get_robot_state2
        gravity_compensation
        # self_collision_checking
        demo_motion
        real_time_control_command

        mobility_command

        rtcomm_protocol
        stop_command

        break_test
        master_arm_utils
        get_pid_gain
        set_pid_gain
        fatory_default_pid_gain

        master_arm_pid_gain
        master_arm_set_default_position_pid_gains
        joystick_test
        whole_body_command
        whole_body_command2
)

# find_package(Qt5Widgets REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Network)
set(CMAKE_AUTOMOC ON)
foreach (SRC IN LISTS EXAMPLES)
    set(EXECUTABLE_NAME example_${SRC})
    add_executable(${EXECUTABLE_NAME}
            ${SRC}.cpp
    )
    target_link_libraries(${EXECUTABLE_NAME} PRIVATE
            rby1-sdk
            DynamixelSDK
            OsqpEigen::OsqpEigen
            Qt5::Widgets
            Qt5::Network
            network
    )
    target_compile_definitions(${EXECUTABLE_NAME} PUBLIC
            -DPATH="${CMAKE_CURRENT_SOURCE_DIR}"
            -DMODELS_PATH="${CMAKE_SOURCE_DIR}/models/"
    )
 
    message(STATUS ${EXECUTABLE_NAME})
    
endforeach ()
