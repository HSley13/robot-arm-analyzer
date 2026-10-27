find_package(Qt6 REQUIRED COMPONENTS Core Gui Qml Quick Quick3D)

# Split out from the executable so arm_controller.cpp is testable with
# plain GTest, without needing a QGuiApplication/QML engine.
add_library(robotarm_app_core
    arm_controller.cpp
)
set_target_properties(robotarm_app_core PROPERTIES AUTOMOC ON)
target_include_directories(robotarm_app_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(robotarm_app_core PUBLIC
    Qt6::Core
    Qt6::Gui
    robotarm_core
    robotarm_ml
)

qt_add_executable(robotarm_app main.cpp)
set_target_properties(robotarm_app PROPERTIES AUTOMOC ON)

target_link_libraries(robotarm_app PRIVATE
    Qt6::Qml
    Qt6::Quick
    Qt6::Quick3D
    robotarm_app_core
)

# Loaded from this real source-tree path at runtime (not bundled into a
# qrc/resource module) so editing a .qml file and reloading the engine -
# see main.cpp's hot-reload watcher - picks it up with no rebuild.
target_compile_definitions(robotarm_app PRIVATE APP_QML_DIR="${CMAKE_CURRENT_SOURCE_DIR}/qml")
