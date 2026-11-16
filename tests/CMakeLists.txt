add_executable(scaffolding_tests scaffolding_test.cpp)
target_link_libraries(scaffolding_tests PRIVATE robotarm_core gtest_main)

add_test(NAME scaffolding_tests COMMAND scaffolding_tests)

add_executable(dh_kinematics_tests dh_kinematics_test.cpp)
target_link_libraries(dh_kinematics_tests PRIVATE robotarm_core gtest_main)

add_test(NAME dh_kinematics_tests COMMAND dh_kinematics_tests)

add_executable(units_tests units_test.cpp)
target_link_libraries(units_tests PRIVATE robotarm_core gtest_main)

add_test(NAME units_tests COMMAND units_tests)

add_executable(statics_tests statics_test.cpp)
target_link_libraries(statics_tests PRIVATE robotarm_core gtest_main)

add_test(NAME statics_tests COMMAND statics_tests)

add_executable(arm_config_tests arm_config_test.cpp)
target_link_libraries(arm_config_tests PRIVATE robotarm_core gtest_main)
target_compile_definitions(arm_config_tests PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")

add_test(NAME arm_config_tests COMMAND arm_config_tests)

add_executable(surrogate_tests surrogate_test.cpp)
target_link_libraries(surrogate_tests PRIVATE robotarm_core robotarm_ml gtest_main)
target_compile_definitions(surrogate_tests PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")

add_test(NAME surrogate_tests COMMAND surrogate_tests)

add_executable(arm_controller_tests arm_controller_test.cpp)
target_link_libraries(arm_controller_tests PRIVATE robotarm_app_core gtest_main)
target_compile_definitions(arm_controller_tests PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")

add_test(NAME arm_controller_tests COMMAND arm_controller_tests)

add_executable(dynamics_tests dynamics_test.cpp)
target_link_libraries(dynamics_tests PRIVATE robotarm_core gtest_main)

add_test(NAME dynamics_tests COMMAND dynamics_tests)

add_executable(trajectory_tests trajectory_test.cpp)
target_link_libraries(trajectory_tests PRIVATE robotarm_core gtest_main)

add_test(NAME trajectory_tests COMMAND trajectory_tests)

add_executable(dynamic_surrogate_tests dynamic_surrogate_test.cpp)
target_link_libraries(dynamic_surrogate_tests PRIVATE robotarm_core robotarm_ml gtest_main)
target_compile_definitions(dynamic_surrogate_tests PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")

add_test(NAME dynamic_surrogate_tests COMMAND dynamic_surrogate_tests)

add_executable(kinematic_tree_tests kinematic_tree_test.cpp)
target_link_libraries(kinematic_tree_tests PRIVATE robotarm_core gtest_main)

add_test(NAME kinematic_tree_tests COMMAND kinematic_tree_tests)

add_executable(recorded_trajectory_tests recorded_trajectory_test.cpp)
target_link_libraries(recorded_trajectory_tests PRIVATE robotarm_core gtest_main)

add_test(NAME recorded_trajectory_tests COMMAND recorded_trajectory_tests)

add_executable(payload_capacity_tests payload_capacity_test.cpp)
target_link_libraries(payload_capacity_tests PRIVATE robotarm_core gtest_main)

add_test(NAME payload_capacity_tests COMMAND payload_capacity_tests)

add_executable(inverse_kinematics_tests inverse_kinematics_test.cpp)
target_link_libraries(inverse_kinematics_tests PRIVATE robotarm_core gtest_main)

add_test(NAME inverse_kinematics_tests COMMAND inverse_kinematics_tests)

add_executable(hand_config_tests hand_config_test.cpp)
target_link_libraries(hand_config_tests PRIVATE robotarm_core gtest_main)
target_compile_definitions(hand_config_tests PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")

add_test(NAME hand_config_tests COMMAND hand_config_tests)

add_executable(hand_gestures_tests hand_gestures_test.cpp)
target_link_libraries(hand_gestures_tests PRIVATE robotarm_core gtest_main)

add_test(NAME hand_gestures_tests COMMAND hand_gestures_tests)

add_executable(hand_controller_tests hand_controller_test.cpp)
target_link_libraries(hand_controller_tests PRIVATE robotarm_app_core gtest_main)
target_compile_definitions(hand_controller_tests PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")

add_test(NAME hand_controller_tests COMMAND hand_controller_tests)

add_executable(pick_planner_tests pick_planner_test.cpp)
target_link_libraries(pick_planner_tests PRIVATE robotarm_core gtest_main)
target_compile_definitions(pick_planner_tests PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")

add_test(NAME pick_planner_tests COMMAND pick_planner_tests)

add_executable(intent_tests intent_test.cpp)
target_link_libraries(intent_tests PRIVATE robotarm_core gtest_main)

add_test(NAME intent_tests COMMAND intent_tests)

add_executable(task_controller_tests task_controller_test.cpp)
target_link_libraries(task_controller_tests PRIVATE robotarm_app_core gtest_main)
target_compile_definitions(task_controller_tests PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")

add_test(NAME task_controller_tests COMMAND task_controller_tests)

# Loads a little QML through the engine, the way the app does: the
# Humanoid.Backend module must register the controllers as singletons.
add_executable(qml_module_tests qml_module_test.cpp)
target_link_libraries(qml_module_tests PRIVATE robotarm_app_core robotarm_app_coreplugin gtest)
target_compile_definitions(qml_module_tests PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")
set_target_properties(qml_module_tests PROPERTIES AUTOMOC ON)

add_test(NAME qml_module_tests COMMAND qml_module_tests)

# The sensor bridge exists only when Qt WebSockets was found (app/CMakeLists.txt).
if(ROBOTARM_HAS_SENSOR_BRIDGE)
  add_executable(sensor_bridge_server_tests sensor_bridge_server_test.cpp)
  target_link_libraries(sensor_bridge_server_tests PRIVATE robotarm_app_core gtest)
  target_compile_definitions(sensor_bridge_server_tests PRIVATE PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}")
  add_test(NAME sensor_bridge_server_tests COMMAND sensor_bridge_server_tests)
endif()

# The webcam control's Python logic (tools/webcam_teleop) - plain unittest,
# no camera or MediaPipe needed. Skipped if there's no Python 3.10+.
find_package(Python3 3.10 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
  add_test(NAME webcam_teleop_tests
           COMMAND ${Python3_EXECUTABLE} -B -m unittest
           WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/tools/webcam_teleop)
endif()
