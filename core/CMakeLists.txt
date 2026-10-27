add_library(robotarm_core
    src/version.cpp
    src/dh.cpp
    src/arm_model.cpp
    src/units.cpp
    src/statics.cpp
    src/arm_config.cpp
    src/dynamics.cpp
    src/trajectory.cpp
)

target_include_directories(robotarm_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(robotarm_core PUBLIC
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
)
