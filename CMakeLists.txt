cmake_minimum_required(VERSION 3.21)
project(HumanoidArmLoadAnalyzer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(nlohmann_json 3.9 REQUIRED)

# Homebrew puts every formula's headers in one shared folder,
# /opt/homebrew/include - which is nlohmann_json's include directory, and
# also holds Homebrew's own Qt (QtCore/, QtQml/, ...) if that's installed.
# Everything linking nlohmann_json then sees that second Qt, and moc (which
# lists Qt's own frameworks last) reads its headers instead of the Qt this
# project builds against - e.g. "Parse error at IID" in the generated QML
# plugin. So give nlohmann_json an include directory that holds nothing
# but nlohmann/.
get_target_property(_json_includes nlohmann_json::nlohmann_json INTERFACE_INCLUDE_DIRECTORIES)
foreach(_dir IN LISTS _json_includes)
  if(EXISTS "${_dir}/nlohmann/json.hpp")
    set(_json_only "${PROJECT_BINARY_DIR}/isolated_include/nlohmann_json")
    file(MAKE_DIRECTORY "${_json_only}")
    file(CREATE_LINK "${_dir}/nlohmann" "${_json_only}/nlohmann" SYMBOLIC COPY_ON_ERROR)
    set_target_properties(nlohmann_json::nlohmann_json PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_json_only}")
    break()
  endif()
endforeach()

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_subdirectory(core)
add_subdirectory(ml)
add_subdirectory(tools)
add_subdirectory(app)

enable_testing()
add_subdirectory(tests)
