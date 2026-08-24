# qp_solver.cpp + qp_backend_osqp.cpp are compiled in directly (they need only
# Eigen + OSQP) so the harness can A/B the refactored backend path against the
# legacy inline OSQP sequence on identical recorded problems.
add_executable(qp_benchmark
        qp_benchmark.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/math/qp_solver.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/math/qp_backend_osqp.cpp
        $<$<BOOL:${RBY1_WITH_PROXQP}>:${CMAKE_CURRENT_SOURCE_DIR}/../src/math/qp_backend_proxqp.cpp>
        $<$<BOOL:${RBY1_WITH_DAQP}>:${CMAKE_CURRENT_SOURCE_DIR}/../src/math/qp_backend_daqp.cpp>
)
target_include_directories(qp_benchmark PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/math
)
target_link_libraries(qp_benchmark PRIVATE Eigen3::Eigen OsqpEigen::OsqpEigen osqp::osqp)
if (RBY1_WITH_PROXQP)
    target_link_libraries(qp_benchmark PRIVATE proxsuite)
    target_compile_definitions(qp_benchmark PRIVATE RBY1_WITH_PROXQP)
endif ()
if (RBY1_WITH_DAQP)
    target_link_libraries(qp_benchmark PRIVATE daqp::daqpstat)
    target_compile_definitions(qp_benchmark PRIVATE RBY1_WITH_DAQP)
endif ()
target_compile_features(qp_benchmark PRIVATE cxx_std_17)

# Gate-5 unit scenario for the sqrt braking envelope (kinematic, no gRPC).
add_executable(test_sqrt_braking
        test_sqrt_braking.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/math/qp_solver.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/math/qp_backend_osqp.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/dynamics/inertial.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/dynamics/joint.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/dynamics/link.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/math/se3.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/math/so3.cpp
)
target_include_directories(test_sqrt_braking PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/math
        ${CMAKE_CURRENT_SOURCE_DIR}/../examples/cpp/dynamics
)
target_link_libraries(test_sqrt_braking PRIVATE Eigen3::Eigen OsqpEigen::OsqpEigen osqp::osqp)
target_compile_features(test_sqrt_braking PRIVATE cxx_std_17)

# Gate-12 unit scenario for the runtime arm distance damper.
add_executable(test_arm_damper
        test_arm_damper.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/math/qp_solver.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/math/qp_backend_osqp.cpp
        $<$<BOOL:${RBY1_WITH_PROXQP}>:${CMAKE_CURRENT_SOURCE_DIR}/../src/math/qp_backend_proxqp.cpp>
        $<$<BOOL:${RBY1_WITH_DAQP}>:${CMAKE_CURRENT_SOURCE_DIR}/../src/math/qp_backend_daqp.cpp>
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/dynamics/inertial.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/dynamics/joint.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/dynamics/link.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/dynamics/robot.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/math/se3.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/math/so3.cpp
)
target_include_directories(test_arm_damper PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/math
)
target_link_libraries(test_arm_damper PRIVATE Eigen3::Eigen tinyxml2::tinyxml2 OsqpEigen::OsqpEigen osqp::osqp)
if (RBY1_WITH_PROXQP)
    target_link_libraries(test_arm_damper PRIVATE proxsuite)
    target_compile_definitions(test_arm_damper PRIVATE RBY1_WITH_PROXQP)
endif ()
if (RBY1_WITH_DAQP)
    target_link_libraries(test_arm_damper PRIVATE daqp::daqpstat)
    target_compile_definitions(test_arm_damper PRIVATE RBY1_WITH_DAQP)
endif ()
target_compile_features(test_arm_damper PRIVATE cxx_std_17)
