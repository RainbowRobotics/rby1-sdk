cmake_minimum_required(VERSION 3.18)
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

set(CMAKE_CXX_STANDARD 17)
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

############################################################
# PROTO
############################################################

set(PROTO_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/_protobuf/${CMAKE_BUILD_TYPE})
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

if (MSVC)
    add_compile_options(/bigobj)
endif ()

configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/version.h.in
        ${CMAKE_CURRENT_BINARY_DIR}/private/rby1-sdk/version.h
)

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
    set_target_properties(rby1-sdk PROPERTIES
            OUTPUT_NAME_DEBUG "rby1-sdk_debug"
            OUTPUT_NAME_RELEASE "rby1-sdk"
    )

    include(GNUInstallDirs)
    set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/rby1-sdk)

    install(TARGETS rby1-sdk
            EXPORT rby1-sdk-targets
            CONFIGURATIONS Debug Release
            RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

    set_target_properties(rby1-sdk PROPERTIES EXPORT_NAME rby1-sdk)

    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/private/rby1-sdk/version.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rby1-sdk)
    install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
    if (INSTALL_MODELS)
        install(DIRECTORY models/ DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/rby1-sdk/models)
    endif ()

    install(EXPORT rby1-sdk-targets
            FILE rby1-sdk-targets.cmake
            NAMESPACE rby1-sdk::
            DESTINATION ${INSTALL_CONFIGDIR})

    include(CMakePackageConfigHelpers)
    configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/rby1-sdk-config.cmake.in
            ${CMAKE_CURRENT_BINARY_DIR}/rby1-sdk-config.cmake
            INSTALL_DESTINATION ${INSTALL_CONFIGDIR})
    install(FILES
            ${CMAKE_CURRENT_BINARY_DIR}/rby1-sdk-config.cmake
            DESTINATION ${INSTALL_CONFIGDIR})
endif ()