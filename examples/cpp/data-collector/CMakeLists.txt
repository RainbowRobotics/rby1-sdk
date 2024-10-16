cmake_minimum_required(VERSION 3.12)
project(data-collector)

find_package(OpenCV REQUIRED)
find_package(cppzmq REQUIRED)
find_package(HDF5 REQUIRED)
find_package(HighFive REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(realsense2 QUIET)
find_package(Boost COMPONENTS filesystem REQUIRED)

add_executable(data-collector
        main.cpp
)
target_include_directories(data-collector PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${HDF5_INCLUDE_DIR}
)
target_link_libraries(data-collector PUBLIC
        rby1-sdk
        cppzmq
        $<${realsense2_FOUND}:realsense2>
        ${OpenCV_LIBRARIES}
#        hdf5
        HDF5::HDF5
        HighFive
        nlohmann_json::nlohmann_json
        Boost::filesystem
)
target_compile_definitions(data-collector PUBLIC
        -DMODELS_PATH="${CMAKE_SOURCE_DIR}/models/"
        $<${realsense2_FOUND}:-DREALSENSE2>
)