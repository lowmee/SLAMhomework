cmake_minimum_required( VERSION 2.8 )
project( Eigentest )

set( CMAKE_BUILD_TYPE "Debug")
set( CMAKE_CXX_FLAGS "-O3")

include_directories( "/usr/include/eigen3" )

add_executable( decomposition decomposition.cpp)
add_executable( pose pose.cpp)
add_executable( why why.cpp)