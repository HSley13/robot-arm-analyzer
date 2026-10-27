find_package(Qt6 REQUIRED COMPONENTS Core Gui Qml Quick Quick3D)

qt_add_executable(robotarm_app main.cpp)
set_target_properties(robotarm_app PROPERTIES AUTOMOC ON)

target_link_libraries(robotarm_app PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Qml
    Qt6::Quick
    Qt6::Quick3D
)

# Loaded from this real source-tree path at runtime (not bundled into a
# qrc/resource module) so editing a .qml file and reloading the engine -
# see main.cpp's hot-reload watcher - picks it up with no rebuild.
target_compile_definitions(robotarm_app PRIVATE APP_QML_DIR="${CMAKE_CURRENT_SOURCE_DIR}/qml")
