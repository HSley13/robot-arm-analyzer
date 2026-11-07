find_package(Qt6 REQUIRED COMPONENTS Core Gui Qml Quick Quick3D QuickControls2 QuickDialogs2)

# Workaround for Qt 6.10.0 and 6.10.1 (fixed in 6.10.2): qt_add_qml_module
# schedules a call on the build directory without quoting its path, so a
# build folder whose path has a space in it - like
# ".../Robot Arm Load Analyzer/build" - fails to configure with
#     cmake_language DEFER DIRECTORY: /Users/.../Robot is not known.
# Qt skips scheduling that call if one with the same ID already exists, so
# schedule the very same call first, with the path quoted (a [==[ ]==]
# bracket argument). The values are baked in now, as Qt does: deferred
# arguments are otherwise only read when the call runs, at the project
# root, where the Qt namespace variable isn't set.
if(Qt6_VERSION VERSION_GREATER_EQUAL 6.10.0 AND Qt6_VERSION VERSION_LESS 6.10.2
   AND COMMAND _qt_internal_write_deferred_qmlls_build_ini_file)
  cmake_language(DEFER DIRECTORY "${CMAKE_BINARY_DIR}" GET_CALL qmlls_build_ini_generation_id _qmlls_call)
  if("${_qmlls_call}" STREQUAL "")
    cmake_language(EVAL CODE
      "cmake_language(DEFER DIRECTORY [==[${CMAKE_BINARY_DIR}]==] ID qmlls_build_ini_generation_id
                      CALL _qt_internal_write_deferred_qmlls_build_ini_file [==[${QT_CMAKE_EXPORT_NAMESPACE}]==])")
  endif()
endif()

# Split out from the executable so the controllers (arm, hand, task)
# are testable with plain GTest, without a window.
add_library(robotarm_app_core STATIC
    arm_controller.cpp
    hand_controller.cpp
    task_controller.cpp
    qml_singleton.hpp
)
set_target_properties(robotarm_app_core PROPERTIES AUTOMOC ON)
target_include_directories(robotarm_app_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(robotarm_app_core PUBLIC
    Qt6::Core
    Qt6::Gui
    Qt6::Qml
    robotarm_core
    robotarm_ml
)

# ...and they're the QML module `Humanoid.Backend`: every class marked
# QML_ELEMENT (+ QML_SINGLETON) becomes a QML type - QML says
# `import Humanoid.Backend` and uses `ArmController.jointCount`,
# `TaskController.pickUp(id)`, signals with `Connections`, ...
# The module holds only C++ types: the UI's .qml files stay outside it,
# loaded from the source folder, so hot reload still works.
#
# A static library's type registrations are linked in through its plugin:
# link `robotarm_app_coreplugin` and put
# `Q_IMPORT_QML_PLUGIN(Humanoid_BackendPlugin)` in the program (main.cpp,
# tests/qml_module_test.cpp).
qt_add_qml_module(robotarm_app_core
    URI Humanoid.Backend
    VERSION 1.0
    # Where Qt 6.5+ looks by default (qrc:/qt/qml); main.cpp adds it as an
    # import path too, for older Qt.
    RESOURCE_PREFIX /qt/qml
    # Type info for qmllint / qmlls (QML completion in an editor) lands here.
    OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/qml/Humanoid/Backend
)

# Procedural meshes (rounded shells, cables) used by the 3D scene. They
# need Qt Quick 3D itself, so they live with the executable rather than in
# robotarm_app_core, which stays Qt-Core/Gui-only for plain GTest.
qt_add_executable(robotarm_app
    main.cpp
    qml_hot_reloader.cpp
    qml_hot_reloader.hpp
    geometry/rounded_box_geometry.cpp
    geometry/rounded_box_geometry.hpp
    geometry/tube_geometry.cpp
    geometry/tube_geometry.hpp
)
set_target_properties(robotarm_app PROPERTIES AUTOMOC ON)

# The procedural meshes are the QML module `Humanoid.Geometry` (classes
# marked QML_ELEMENT). The program itself backs it, so its types are
# registered just by being linked in - no qmlRegisterType() calls.
qt_add_qml_module(robotarm_app
    URI Humanoid.Geometry
    VERSION 1.0
    RESOURCE_PREFIX /qt/qml
    OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/qml/Humanoid/Geometry
)
# (The generated registration code includes the headers by bare name.)
target_include_directories(robotarm_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/geometry)

target_link_libraries(robotarm_app PRIVATE
    robotarm_app_coreplugin
    Qt6::Qml
    Qt6::Quick
    Qt6::Quick3D
    Qt6::QuickControls2
    Qt6::QuickDialogs2
    robotarm_app_core
)

# Loaded from this real source-tree path at runtime (not bundled into a
# qrc/resource module) so editing a .qml file and reloading the engine -
# see QmlHotReloader - picks it up with no rebuild.
target_compile_definitions(robotarm_app PRIVATE
    APP_QML_DIR="${CMAKE_CURRENT_SOURCE_DIR}/qml"
    PROJECT_ROOT_DIR="${CMAKE_SOURCE_DIR}"
)
