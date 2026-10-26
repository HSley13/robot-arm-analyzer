add_library(robotarm_core
    src/version.cpp
)

target_include_directories(robotarm_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(robotarm_core PUBLIC
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
)
