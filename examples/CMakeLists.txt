add_subdirectory(dynamics)
add_subdirectory(module_test)

set(EXAMPLES
        trapezoidal_motion_generator
        so3
        optimal_control
        udp_test
        velocity_estimation

        master_arm
        teleoperation_with_joint_mapping

        kinematic_calibration
        kinematic_calibration2

        log
        reset_odometry
        power_command

        get_robot_state
        get_robot_state2
        gravity_compensation
        self_collision_checking
        demo_motion

        mobility_command
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
    )
    target_compile_definitions(${EXECUTABLE_NAME} PUBLIC
            -DPATH="${CMAKE_CURRENT_SOURCE_DIR}"
    )
endforeach ()

#add_executable(example_rtcomm_protocol
#        rtcomm_protocol.cpp
#)
#target_link_libraries(example_rtcomm_protocol PRIVATE
#        rby1-sdk
#)