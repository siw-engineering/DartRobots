include(CMakeFindDependencyMacro)
find_dependency(Eigen3)
find_dependency(spdlog)
find_dependency(DART COMPONENTS utils-urdf gui-osg collision-ode)

include("${CMAKE_CURRENT_LIST_DIR}/DartRobotsTargets.cmake")