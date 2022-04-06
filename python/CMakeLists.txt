find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
find_package(pybind11 REQUIRED)

pybind11_add_module(MiniCheetahPy MiniCheetahPy.cpp)


set_target_properties(MiniCheetahPy PROPERTIES
        INTERPROCEDURAL_OPTIMIZATION TRUE
        POSITION_INDEPENDENT_CODE ON
        INSTALL_RPATH "$ORIGIN/../../../../lib")
target_include_directories(MiniCheetahPy PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(MiniCheetahPy PRIVATE MiniCheetah)

install(TARGETS MiniCheetahPy
        LIBRARY DESTINATION DartRobots
        )