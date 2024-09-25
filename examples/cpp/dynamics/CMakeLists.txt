set(DYNAMICS_EXAMPLES
        inverse_dynamics
        mass_matrix
        gravity_term
        dynamics_coeff
        reflective_mass
        simple_robot
        load_urdf
)

foreach (SRC IN LISTS DYNAMICS_EXAMPLES)
    set(EXECUTABLE_NAME example_dynamics_${SRC})
    add_executable(${EXECUTABLE_NAME}
        ${SRC}.cpp
    )
    target_link_libraries(${EXECUTABLE_NAME} PRIVATE rby1-sdk)
    target_compile_definitions(${EXECUTABLE_NAME} PUBLIC
            -DPATH="${CMAKE_CURRENT_SOURCE_DIR}"
    )
endforeach ()