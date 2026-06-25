cmake_minimum_required(VERSION 3.20)
project(MotionStudio VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets)

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

set(SOURCES
    src/main.cpp
    src/MainWindow.cpp
    src/viewport/Camera.cpp
    src/viewport/Viewport.cpp
    src/scene/SceneGraph.cpp
    src/physics/SimulationEngine.cpp
    src/ui/HierarchyPanel.cpp
    src/ui/PropertiesPanel.cpp
    src/serialization/ProjectSerializer.cpp
)

add_executable(MotionStudio ${SOURCES})

target_include_directories(MotionStudio PRIVATE src)

target_link_libraries(MotionStudio
    PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    nlohmann_json::nlohmann_json
)

if(MUJOCO_FOUND)
    target_link_libraries(MotionStudio PRIVATE mujoco)
    target_compile_definitions(MotionStudio PRIVATE MUJOCO_ENABLED)
endif()
