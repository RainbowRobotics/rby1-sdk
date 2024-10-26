cmake_minimum_required(VERSION 3.28)
if (NOT DEFINED SKBUILD_PROJECT_VERSION)
    file(READ ${CMAKE_CURRENT_SOURCE_DIR}/pyproject.toml PYPROJECT_STR)
    string(REGEX MATCHALL "[^\n\r]+" PYPROJECT_STR_LIST "${PYPROJECT_STR}")

    foreach (STR ${PYPROJECT_STR_LIST})
        if (${STR} MATCHES "^version[ \t\r\n]+=[ \t\r\n]+\"(.*)\"")
            set(RBY1_SDK_VERSION ${CMAKE_MATCH_1})
        endif ()
    endforeach ()
else ()
    set(RBY1_SDK_VERSION ${SKBUILD_PROJECT_VERSION})
endif ()
project(rby1-sdk
        VERSION ${RBY1_SDK_VERSION}
        LANGUAGES CXX)

if (NOT DEFINED CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()
message(STATUS "RBY1-SDK Version: " ${PROJECT_VERSION})

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

############################################################
# OPTIONS
############################################################

set(BUILD_EXAMPLES OFF CACHE BOOL "Build examples")
set(BUILD_LIB_STATIC OFF CACHE BOOL "Build static rby1_sdk library")
set(BUILD_PYTHON_BINDINGS OFF CACHE BOOL "Build Python bindings using Pybind11")
set(INSTALL_MODELS ON CACHE BOOL "Install library with models")

message(STATUS "RBY1-SDK: Build examples: " ${BUILD_EXAMPLES})

############################################################
# DEPENDENCIES
############################################################

find_package(gRPC REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(nlohmann_json REQUIRED)

include(FetchContent)

set(OSQP_BUILD_DEMO_EXE OFF)

FetchContent_Declare(osqp
        GIT_REPOSITORY https://github.com/osqp/osqp
        GIT_TAG ff371613bef82e85fb431590343853846ff96203
        OVERRIDE_FIND_PACKAGE
        EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(osqp)
add_library(osqp::osqp INTERFACE IMPORTED)
target_link_libraries(osqp::osqp INTERFACE osqpstatic)

FetchContent_Declare(OsqpEigen
        GIT_REPOSITORY https://github.com/robotology/osqp-eigen
        GIT_TAG b9ce7391350e0f1c71e49f7574a8785fb617b5e6
        OVERRIDE_FIND_PACKAGE
        EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(OsqpEigen)
target_compile_definitions(OsqpEigen PUBLIC -DOSQP_EIGEN_OSQP_IS_V1)

############################################################
# PROTO
############################################################

set(PROTO_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/_protobuf)
set(PROTO_IMPORT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/protos)
file(MAKE_DIRECTORY ${PROTO_BINARY_DIR})
file(GLOB_RECURSE PROTO_FILES ${PROTO_IMPORT_DIR}/*.proto)
add_library(proto-objects OBJECT ${PROTO_FILES})
target_link_libraries(proto-objects PUBLIC protobuf::libprotobuf gRPC::grpc++)
target_include_directories(proto-objects PUBLIC ${PROTO_BINARY_DIR})
protobuf_generate(
        TARGET proto-objects
        IMPORT_DIRS ${PROTO_IMPORT_DIR}
        PROTOC_OUT_DIR ${PROTO_BINARY_DIR}
        USAGE_REQUIREMENT INTERFACE
)
protobuf_generate(
        TARGET proto-objects
        LANGUAGE grpc
        GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc
        PLUGIN "protoc-gen-grpc=\$<TARGET_FILE:gRPC::grpc_cpp_plugin>"
        USAGE_REQUIREMENT INTERFACE
        IMPORT_DIRS ${PROTO_IMPORT_DIR}
        PROTOC_OUT_DIR ${PROTO_BINARY_DIR})
############################################################

add_subdirectory(third-party)
add_subdirectory(src)

find_package(Threads REQUIRED)

if (BUILD_EXAMPLES)
    add_subdirectory(examples/cpp)
#     target_link_libraries(example_master_arm PRIVATE Threads::Threads)
    target_link_libraries(example_teleoperation_with_joint_mapping PRIVATE Threads::Threads)
endif ()

############################################################
# PYTHON BINDING
############################################################

if (BUILD_PYTHON_BINDINGS)
    add_subdirectory(python)
endif ()

############################################################
# INSTALL
############################################################

if ((NOT BUILD_PYTHON_BINDINGS)
        AND (${PROJECT_NAME} STREQUAL ${CMAKE_PROJECT_NAME})
)
    include(GNUInstallDirs)
    set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/rby1-sdk)

    install(TARGETS rby1-sdk
            EXPORT rby1-sdk-targets
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

    set_target_properties(rby1-sdk PROPERTIES EXPORT_NAME rby1-sdk)

    install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
    if (INSTALL_MODELS)
        install(DIRECTORY models/ DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/rby1-sdk/models)
    endif ()

    install(EXPORT rby1-sdk-targets
            FILE
            rby1-sdk-targets.cmake
            NAMESPACE
            rby1-sdk::
            DESTINATION
            ${INSTALL_CONFIGDIR})

    include(CMakePackageConfigHelpers)
    configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/rby1-sdk-config.cmake.in
            ${CMAKE_CURRENT_BINARY_DIR}/rby1-sdk-config.cmake
            INSTALL_DESTINATION ${INSTALL_CONFIGDIR})
    install(FILES
            ${CMAKE_CURRENT_BINARY_DIR}/rby1-sdk-config.cmake
            DESTINATION ${INSTALL_CONFIGDIR})
endif ()