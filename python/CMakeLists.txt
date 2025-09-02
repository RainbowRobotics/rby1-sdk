if (${CMAKE_VERSION} VERSION_GREATER_EQUAL 3.27)
    cmake_policy(SET CMP0148 OLD)
endif ()

if (${CMAKE_VERSION} VERSION_GREATER_EQUAL 3.24)
    cmake_policy(SET CMP0135 OLD)
endif ()

find_package(Python 3 COMPONENTS Interpreter Development REQUIRED)

include(FetchContent)
FetchContent_Declare(
        pybind11
        URL https://github.com/pybind/pybind11/archive/refs/tags/v2.13.5.tar.gz
        URL_HASH
        SHA256=b1e209c42b3a9ed74da3e0b25a4f4cd478d89d5efbb48f04b277df427faf6252)
FetchContent_MakeAvailable(pybind11)

if (${CMAKE_VERSION} VERSION_GREATER_EQUAL 3.27)
    cmake_policy(SET CMP0148 NEW)
endif ()

if (${CMAKE_VERSION} VERSION_GREATER_EQUAL 3.24)
    cmake_policy(SET CMP0135 NEW)
endif ()

pybind11_add_module(_bindings
        rby1_sdk.cpp
        math.cpp
        common.cpp
        net.cpp
        model.cpp
        robot_state.cpp
        robot_command_builder.cpp
        robot.cpp
        robot_info.cpp
        log.cpp
        robot_command_feedback.cpp
        control_manager_state.cpp
        dynamics.cpp
        dynamixel_bus.cpp
        upc.cpp
        print_helper.cpp
)
target_link_libraries(_bindings PRIVATE rby1-sdk Eigen3::Eigen)

target_compile_options(
        _bindings PRIVATE "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall>")
target_compile_definitions(_bindings PRIVATE PYBIND11_DETAILED_ERROR_MESSAGES)
# move the binary `_bindings` to its destination in the Python module

set(BINDING_INSTALL_DIR "./python/rby1_sdk/")

install(TARGETS _bindings DESTINATION ${BINDING_INSTALL_DIR})

add_custom_target(generate_stub ALL
        COMMAND ${Python_EXECUTABLE} "-m" "pybind11_stubgen" "--numpy-array-use-type-var" "-o${CMAKE_INSTALL_PREFIX}/${BINDING_INSTALL_DIR}" "_bindings"
        WORKING_DIRECTORY $<TARGET_FILE_DIR:_bindings>
        DEPENDS _bindings
)