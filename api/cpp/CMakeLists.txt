# MotionStudio robot API for C++. Use it from your project with
#   add_subdirectory(path/to/api/cpp)          (or FetchContent)
#   target_link_libraries(your_app PRIVATE motionstudio::motionstudio)
cmake_minimum_required(VERSION 3.16)
project(motionstudio_cpp VERSION 0.3.0 LANGUAGES CXX)

option(MOTIONSTUDIO_CPP_EXAMPLES "Build the examples" ${PROJECT_IS_TOP_LEVEL})
option(MOTIONSTUDIO_CPP_TESTS "Build the live test" ${PROJECT_IS_TOP_LEVEL})

find_package(Threads REQUIRED)

add_library(motionstudio src/motionstudio.cpp)
add_library(motionstudio::motionstudio ALIAS motionstudio)
target_include_directories(motionstudio PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(motionstudio PUBLIC cxx_std_17)
target_link_libraries(motionstudio PRIVATE Threads::Threads)
if(WIN32)
    target_link_libraries(motionstudio PRIVATE ws2_32)
endif()

if(MOTIONSTUDIO_CPP_EXAMPLES)
    foreach(ex square humanoid_wave two_motions walk_square walk_patrol)
        add_executable(${ex} examples/${ex}.cpp)
        target_link_libraries(${ex} PRIVATE motionstudio::motionstudio)
        set_target_properties(${ex} PROPERTIES CXX_STANDARD 20)
    endforeach()
endif()

if(MOTIONSTUDIO_CPP_TESTS)
    add_executable(live_test tests/live_test.cpp)
    target_link_libraries(live_test PRIVATE motionstudio::motionstudio)
endif()

include(GNUInstallDirs)
install(TARGETS motionstudio EXPORT motionstudioTargets)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT motionstudioTargets NAMESPACE motionstudio:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/motionstudio
        FILE motionstudioConfig.cmake)
