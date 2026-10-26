cmake_minimum_required(VERSION 3.21)
project(RobotArmLoadAnalyzer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(nlohmann_json 3.9 REQUIRED)

add_subdirectory(core)

enable_testing()
add_subdirectory(tests)
