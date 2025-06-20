if (BUILD_PYTHON_BINDINGS OR BUILD_LIB_STATIC)
    set(LIBRARY_TYPE STATIC)
else ()
    set(LIBRARY_TYPE SHARED)
endif ()
message(STATUS "Building library as ${LIBRARY_TYPE} (${CMAKE_SYSTEM_NAME})")

add_library(rby1-sdk ${LIBRARY_TYPE}
        robot.cpp
        robot_command_builder.cpp
        robot_command_feedback.cpp

        base/dynamixel_bus.cpp

        math/qp_solver.cpp
        math/se2.cpp
        math/so3.cpp
        math/se3.cpp
        math/trapezoidal_motion_generator.cpp

        dynamics/inertial.cpp
        dynamics/link.cpp
        dynamics/joint.cpp
        dynamics/robot.cpp

        upc/device.cpp
        upc/master_arm.cpp

        net/real_time_control_protocol.cpp

        $<TARGET_OBJECTS:proto-objects>
)
target_link_libraries(rby1-sdk PUBLIC
        Eigen3::Eigen
)
target_link_libraries(rby1-sdk PRIVATE
        osqp::osqp
        OsqpEigen::OsqpEigen
        tinyxml2::tinyxml2
        gRPC::grpc++
        protobuf::libprotobuf
        nlohmann_json::nlohmann_json
        DynamixelSDK
)
if(WIN32)
    target_link_libraries(rby1-sdk PUBLIC wsock32 ws2_32)
endif()

if (LIBRARY_TYPE STREQUAL "STATIC")
    target_compile_definitions(rby1-sdk PUBLIC RBY1_SDK_STATIC)
endif()
if (LIBRARY_TYPE STREQUAL "SHARED")
    target_compile_definitions(rby1-sdk PRIVATE RBY1_SDK_EXPORTS)
endif()

target_include_directories(rby1-sdk PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../private/>
        $<BUILD_INTERFACE:${PROTO_BINARY_DIR}>)