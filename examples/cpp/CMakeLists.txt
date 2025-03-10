add_subdirectory(dynamics)
add_subdirectory(module_test)

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
        get_pid_gain
        set_pid_gain
        fatory_default_pid_gain

        demo_motion_with_master_arm
        wholebody_control_with_joystick
)

foreach (SRC IN LISTS EXAMPLES)
    set(EXECUTABLE_NAME example_${SRC})
    if (${SRC} STREQUAL "wholebody_control_with_joystick")
        add_executable(${EXECUTABLE_NAME}
                ${SRC}.cpp
                joystick/joystick_receiver.cpp 
        )
        target_include_directories(${EXECUTABLE_NAME} PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}/joystick
        )
    else()
        add_executable(${EXECUTABLE_NAME}
                ${SRC}.cpp
        )
    endif()
    
    
    target_link_libraries(${EXECUTABLE_NAME} PRIVATE
            rby1-sdk
            DynamixelSDK
            OsqpEigen::OsqpEigen
    )
    target_compile_definitions(${EXECUTABLE_NAME} PUBLIC
            -DPATH="${CMAKE_CURRENT_SOURCE_DIR}"
            -DMODELS_PATH="${CMAKE_SOURCE_DIR}/models/"
    )
    message(STATUS ${EXECUTABLE_NAME})
endforeach ()
