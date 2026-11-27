# MotionStudio robot API for C++17. Ways to use it from your project:
#   find_package(motionstudio 0.3 REQUIRED)       after cmake --install (see README)
#   FetchContent with the release archive          motionstudio-cpp-<version>.tar.gz
#   add_subdirectory(path/to/motionstudio-cpp)     a copy in your source tree
# then: target_link_libraries(your_app PRIVATE motionstudio::motionstudio)
# Without CMake: pkg-config --cflags --libs motionstudio
cmake_minimum_required(VERSION 3.16)
project(motionstudio_cpp VERSION 0.3.0 LANGUAGES CXX)

if(NOT DEFINED PROJECT_IS_TOP_LEVEL)   # CMake < 3.21
    string(COMPARE EQUAL "${CMAKE_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}" PROJECT_IS_TOP_LEVEL)
endif()
option(MOTIONSTUDIO_CPP_EXAMPLES "Build the examples" ${PROJECT_IS_TOP_LEVEL})
option(MOTIONSTUDIO_CPP_TESTS "Build the live test" ${PROJECT_IS_TOP_LEVEL})
option(MOTIONSTUDIO_CPP_INSTALL "Generate the install rules" ${PROJECT_IS_TOP_LEVEL})

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
set_target_properties(motionstudio PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    POSITION_INDEPENDENT_CODE ON
    WINDOWS_EXPORT_ALL_SYMBOLS ON)

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

if(MOTIONSTUDIO_CPP_INSTALL)
    include(GNUInstallDirs)
    include(CMakePackageConfigHelpers)
    set(config_dir ${CMAKE_INSTALL_LIBDIR}/cmake/motionstudio)

    install(TARGETS motionstudio EXPORT motionstudioTargets
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
    install(EXPORT motionstudioTargets NAMESPACE motionstudio:: DESTINATION ${config_dir})

    configure_package_config_file(cmake/motionstudioConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/motionstudioConfig.cmake INSTALL_DESTINATION ${config_dir})
    write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/motionstudioConfigVersion.cmake
        COMPATIBILITY SameMinorVersion)   # 0.x: a new minor version may change the API
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/motionstudioConfig.cmake
                  ${CMAKE_CURRENT_BINARY_DIR}/motionstudioConfigVersion.cmake
            DESTINATION ${config_dir})

    # pkg-config, for Makefiles and other build systems
    # Paths relative to the .pc file, so an install can be moved (or made with --prefix)
    file(RELATIVE_PATH pc_up "/p/${CMAKE_INSTALL_LIBDIR}/pkgconfig" "/p")
    string(REGEX REPLACE "/$" "" pc_up "${pc_up}")
    set(PC_PREFIX "\${pcfiledir}/${pc_up}")
    set(PC_LIBDIR "\${prefix}/${CMAKE_INSTALL_LIBDIR}")
    set(PC_INCLUDEDIR "\${prefix}/${CMAKE_INSTALL_INCLUDEDIR}")
    if(WIN32)
        set(PC_PRIVATE_LIBS "-lws2_32")
    else()
        set(PC_PRIVATE_LIBS "-pthread")
    endif()
    configure_file(cmake/motionstudio.pc.in ${CMAKE_CURRENT_BINARY_DIR}/motionstudio.pc @ONLY)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/motionstudio.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
    install(FILES LICENSE README.md DESTINATION ${CMAKE_INSTALL_DOCDIR})
endif()
