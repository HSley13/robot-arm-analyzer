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
