add_library(robotarm_ml
    src/surrogate.cpp
)

target_include_directories(robotarm_ml PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(robotarm_ml PUBLIC
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
)
