###############################################################
# DynamixelSDK
###############################################################
if (NOT TARGET DynamixelSDK)
    set(DynamixelSDK_SRC_PATH ${CMAKE_CURRENT_SOURCE_DIR}/DynamixelSDK/c++/src/dynamixel_sdk)
    set(DynamixelSDK_SRCS
            ${DynamixelSDK_SRC_PATH}/group_bulk_read.cpp
            ${DynamixelSDK_SRC_PATH}/group_bulk_write.cpp
            ${DynamixelSDK_SRC_PATH}/group_fast_bulk_read.cpp
            ${DynamixelSDK_SRC_PATH}/group_fast_sync_read.cpp
            ${DynamixelSDK_SRC_PATH}/group_handler.cpp
            ${DynamixelSDK_SRC_PATH}/group_sync_read.cpp
            ${DynamixelSDK_SRC_PATH}/group_sync_write.cpp
            ${DynamixelSDK_SRC_PATH}/packet_handler.cpp
            ${DynamixelSDK_SRC_PATH}/port_handler.cpp
            ${DynamixelSDK_SRC_PATH}/protocol1_packet_handler.cpp
            ${DynamixelSDK_SRC_PATH}/protocol2_packet_handler.cpp
    )
    if (APPLE)
        set(DynamixelSDK_SRCS
                ${DynamixelSDK_SRCS}
                ${DynamixelSDK_SRC_PATH}/port_handler_mac.cpp
        )
    elseif (WIN32)
        set(DynamixelSDK_SRCS
                ${DynamixelSDK_SRCS}
                ${DynamixelSDK_SRC_PATH}/port_handler_windows.cpp
        )
    else ()
        set(DynamixelSDK_SRCS
                ${DynamixelSDK_SRCS}
                ${DynamixelSDK_SRC_PATH}/port_handler_linux.cpp
        )
    endif ()
    add_library(DynamixelSDK
            ${DynamixelSDK_SRCS}
    )
    target_include_directories(DynamixelSDK PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/DynamixelSDK/c++/include/dynamixel_sdk)

    set_target_properties(DynamixelSDK PROPERTIES
            CXX_VISIBILITY_PRESET hidden
            VISIBILITY_INLINES_HIDDEN YES
    )
endif ()

###############################################################
# OSQP
###############################################################
set(OSQP_BUILD_DEMO_EXE OFF)
set(OSQP_BUILD_SHARED_LIB OFF)
set(PROFILING ON)
set(CMAKE_SKIP_INSTALL_RULES ON)
add_subdirectory(osqp EXCLUDE_FROM_ALL)
set(CMAKE_SKIP_INSTALL_RULES OFF)
add_library(osqp::osqp ALIAS osqpstatic)

###############################################################
# OSQP-Eigen
###############################################################
set(OSQP_IS_V1 ON)
set(OSQP_IS_V1_FINAL ON)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_SKIP_INSTALL_RULES ON)
add_subdirectory(osqp-eigen EXCLUDE_FROM_ALL)
set(CMAKE_SKIP_INSTALL_RULES OFF)
target_compile_definitions(OsqpEigen PUBLIC PROFILING)