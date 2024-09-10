set(MODULE_TEST
        head
        torso
        right_arm
        left_arm
        ft_sensor 
        master_arm
)

foreach (SRC IN LISTS MODULE_TEST)
    set(EXECUTABLE_NAME module_test_${SRC})
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
