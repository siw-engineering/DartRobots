find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
find_package(pybind11 REQUIRED)

pybind11_add_module(DartRobotsPy DartRobotsPy.cpp)


set_target_properties(DartRobotsPy PROPERTIES
        INTERPROCEDURAL_OPTIMIZATION TRUE
        POSITION_INDEPENDENT_CODE ON
        INSTALL_RPATH "$ORIGIN/../../../../lib")
target_include_directories(DartRobotsPy PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(DartRobotsPy PRIVATE MiniCheetah)

install(TARGETS DartRobotsPy
        LIBRARY DESTINATION DartRobots
        )