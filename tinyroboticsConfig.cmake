# tinyroboticsConfig.cmake

include(CMakeFindDependencyMacro)

find_dependency(Catch2)
find_dependency(Eigen3)
find_dependency(TinyXML2)

include("${CMAKE_CURRENT_LIST_DIR}/tinyroboticsTargets.cmake")
