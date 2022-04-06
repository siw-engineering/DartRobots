cmake_minimum_required(VERSION 3.16)
project(DartRobots)

# Enforce LTO
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Release build" FORCE)
endif ()

if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # enable warnings using Clang or GCC
    add_compile_options(-Wall -Wextra -pedantic -Wno-unknown-pragmas)
endif ()

########### Add uninstall target ###############
# Taken from https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/recipe/AddUninstallTarget
if (NOT TARGET uninstall)
    configure_file(
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
            "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
            IMMEDIATE @ONLY)
    add_custom_target(uninstall
            "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake")
endif ()

# Allows setting CMAKE_INSTALL_FULL_DATADIR during development to use source directory for resource path
if (DEFINED CMAKE_INSTALL_FULL_DATADIR)
    set(TEMP_VAR ${CMAKE_INSTALL_FULL_DATADIR})
    include(GNUInstallDirs)
    set(CMAKE_INSTALL_FULL_DATADIR ${TEMP_VAR})
else ()
    include(GNUInstallDirs)
endif ()

option(PACKAGE_PYTHON "Install as part of a python package, affects install paths" OFF)

set(CMAKE_CXX_STANDARD 20)

find_package(DART 6.9.2 REQUIRED COMPONENTS utils-urdf gui-osg collision-ode)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)


# Set such that the configure header for resource paths use relative path
# This is fine if the library only gets used by the python interpreter which is always the same path
if (PACKAGE_PYTHON)
    set(CMAKE_INSTALL_FULL_DATADIR_TEMP ${CMAKE_INSTALL_FULL_DATADIR})
    set(CMAKE_INSTALL_FULL_DATADIR "../share")
endif ()
set(config_header_in Config.hpp.in)
set(config_header_out ${CMAKE_CURRENT_SOURCE_DIR}/include/${CMAKE_PROJECT_NAME}/Config.hpp)
configure_file(${config_header_in} ${config_header_out})

if (PACKAGE_PYTHON)
    set(CMAKE_INSTALL_FULL_DATADIR ${CMAKE_INSTALL_FULL_DATADIR_TEMP})
endif ()

add_library(MiniCheetah src/MiniCheetah.cpp src/detail/MiniCheetahImpl.cpp src/detail/Helpers.cpp)
set_target_properties(MiniCheetah PROPERTIES
        INTERPROCEDURAL_OPTIMIZATION TRUE
        POSITION_INDEPENDENT_CODE ON)
target_link_libraries(MiniCheetah PUBLIC dart-gui-osg dart-collision-ode
        dart-utils-urdf dart)
target_include_directories(MiniCheetah PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

add_subdirectory(python)

install(
        TARGETS MiniCheetah EXPORT MiniCheetahConfig
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY resources/
        DESTINATION ${CMAKE_INSTALL_DATADIR}/${CMAKE_PROJECT_NAME}/resources)
install(EXPORT MiniCheetahConfig NAMESPACE DartRobots::
        FILE "${CMAKE_PROJECT_NAME}Targets.cmake"
        DESTINATION "${CMAKE_INSTALL_DATADIR}/${CMAKE_PROJECT_NAME}")
install(
        FILES cmake/install-config.cmake
        DESTINATION "${CMAKE_INSTALL_DATADIR}/${CMAKE_PROJECT_NAME}"
        RENAME "${CMAKE_PROJECT_NAME}Config.cmake"
)
