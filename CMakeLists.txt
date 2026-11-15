cmake_minimum_required(VERSION 3.20)
project(MotionStudio VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets Concurrent Network Xml)

# nlohmann/json via FetchContent
include(FetchContent)
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(nlohmann_json)

# MuJoCo — point to the extracted pre-built archive
# Default: ~/mujoco-3.10.0  Override with -DMUJOCO_ROOT=<path>
set(MUJOCO_ROOT "$ENV{HOME}/mujoco-3.10.0" CACHE PATH "Path to extracted MuJoCo directory")

if(EXISTS "${MUJOCO_ROOT}/include/mujoco/mujoco.h")
    message(STATUS "MuJoCo found at ${MUJOCO_ROOT}")
    add_library(mujoco SHARED IMPORTED)
    set_target_properties(mujoco PROPERTIES
        IMPORTED_LOCATION "${MUJOCO_ROOT}/lib/libmujoco.so.3.10.0"
        INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_ROOT}/include"
    )
    set(MUJOCO_FOUND TRUE)
else()
    message(WARNING "MuJoCo not found at ${MUJOCO_ROOT}. Simulation will use stub backend.")
    set(MUJOCO_FOUND FALSE)
endif()

# OpenCASCADE (OCCT) — STEP import. Optional, same pattern as MuJoCo:
# without it, STEP import is compiled out (no OCCT_ENABLED define).
find_path(OCCT_INCLUDE_DIR NAMES opencascade/STEPCAFControl_Reader.hxx)
find_library(OCCT_TKXDESTEP_LIB NAMES TKXDESTEP)
if(OCCT_INCLUDE_DIR AND OCCT_TKXDESTEP_LIB)
    message(STATUS "OpenCASCADE found (${OCCT_TKXDESTEP_LIB}) — STEP import enabled")
    set(OCCT_FOUND TRUE)
else()
    message(WARNING "OpenCASCADE not found. STEP import will be disabled. "
                    "Install libocct-data-exchange-dev, libocct-ocaf-dev, "
                    "libocct-modeling-algorithms-dev to enable it.")
    set(OCCT_FOUND FALSE)
endif()

# Everything that does not need widgets or OpenGL: scene, automation,
# physics, file I/O. The app and the (optional) tests link it.
add_library(MotionStudioCore STATIC
    src/scene/SceneGraph.cpp
    src/scene/UndoManager.cpp
    src/scene/MeshWeld.cpp
    src/scene/PrimitiveMesh.cpp
    src/scene/SceneQuery.cpp
    src/scene/Interference.cpp
    src/scene/SceneMerge.cpp
    src/scene/Assembly.cpp
    src/physics/SimulationEngine.cpp
    src/physics/Recording.cpp
    src/automation/Expression.cpp
    src/automation/AutomationEngine.cpp
    src/automation/Sensors.cpp
    src/automation/Kinematics.cpp
    src/serialization/ProjectSerializer.cpp
    src/import/StepImporter.cpp
    src/io/ModbusServer.cpp
    src/io/RobotFormats.cpp
    src/robot/RobotBuilder.cpp
    src/robot/RobotAnalysis.cpp
    src/robot/HumanoidBuilder.cpp
)
target_include_directories(MotionStudioCore PUBLIC src)
target_link_libraries(MotionStudioCore PUBLIC Qt6::Core Qt6::Gui Qt6::Network Qt6::Xml)

if(MUJOCO_FOUND)
    target_link_libraries(MotionStudioCore PUBLIC mujoco)
    target_compile_definitions(MotionStudioCore PUBLIC MUJOCO_ENABLED)
endif()

if(OCCT_FOUND)
    target_include_directories(MotionStudioCore PRIVATE "${OCCT_INCLUDE_DIR}/opencascade")
    get_filename_component(OCCT_LIB_DIR "${OCCT_TKXDESTEP_LIB}" DIRECTORY)
    target_link_directories(MotionStudioCore PUBLIC "${OCCT_LIB_DIR}")
    target_link_libraries(MotionStudioCore PUBLIC
        TKernel TKMath TKG2d TKG3d TKGeomBase TKBRep
        TKGeomAlgo TKTopAlgo TKShHealing TKMesh
        TKXSBase TKSTEPBase TKSTEPAttr TKSTEP209 TKSTEP
        TKCDF TKLCAF TKCAF TKXCAF TKXDESTEP
    )
    target_compile_definitions(MotionStudioCore PUBLIC OCCT_ENABLED)
endif()

set(SOURCES
    src/main.cpp
    src/MainWindow.cpp
    src/viewport/Camera.cpp
    src/viewport/Viewport.cpp
    src/viewport/LightingPanel.cpp
    src/viewport/SectionPanel.cpp
    src/viewport/AssemblePanel.cpp
    src/viewport/ViewCube.cpp
    src/viewport/MeshFeatures.cpp
    src/ui/HierarchyPanel.cpp
    src/ui/PropertiesPanel.cpp
    src/ui/TemplateDialog.cpp
    src/ui/HelpDialog.cpp
    src/ui/ImportProgressDialog.cpp
    src/ui/Theme.cpp
    src/ui/Island.cpp
    src/ui/TimelineBar.cpp
    src/ui/ModbusDialog.cpp
    src/ui/robot/RobotSketch.cpp
    src/ui/robot/RobotAnalysisPanel.cpp
    src/ui/robot/RobotBuilderDialog.cpp
    src/ui/robot/RobotAnalyzerDialog.cpp
    src/ui/robot/HumanoidBuilderDialog.cpp
    src/ui/automation/AutomationCommon.cpp
    src/ui/automation/AutomationPanel.cpp
    src/ui/automation/ExpressionEdit.cpp
    src/ui/automation/SignalsPage.cpp
    src/ui/automation/SensorDialog.cpp
    src/ui/automation/PartFlowPage.cpp
    src/ui/automation/ActuatorsPage.cpp
    src/ui/automation/SequencesPage.cpp
    src/ui/automation/RuntimePage.cpp
    src/ui/automation/GanttView.cpp
    src/ui/automation/SignalTracePlot.cpp
    resources.qrc
)

add_executable(MotionStudio ${SOURCES})

target_link_libraries(MotionStudio
    PRIVATE
    MotionStudioCore
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    Qt6::Concurrent
    Qt6::Network
    nlohmann_json::nlohmann_json
)

# Headless regression tests for the core: cmake -DMOTIONSTUDIO_TESTS=ON,
# then ./build/MotionStudioTests (exit code 0 = all passed)
option(MOTIONSTUDIO_TESTS "Build the core regression tests" OFF)
if(MOTIONSTUDIO_TESTS)
    add_executable(MotionStudioTests tests/main.cpp)
    target_link_libraries(MotionStudioTests PRIVATE MotionStudioCore)
    target_compile_definitions(MotionStudioTests PRIVATE MS_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
    if(OCCT_FOUND)
        # The STEP import test writes its own small assembly with OCCT
        target_include_directories(MotionStudioTests PRIVATE "${OCCT_INCLUDE_DIR}/opencascade")
        target_link_libraries(MotionStudioTests PRIVATE TKPrim)
    endif()
endif()
