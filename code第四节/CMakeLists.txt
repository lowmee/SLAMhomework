cmake_minimum_required( VERSION 2.8)
project( camera )

set( CMAKE_CXX_FLAGS "-std=c++11" )

find_package( OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})

find_package( PCL REQUIRED COMPONENT common io )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

find_package( Pangolin REQUIRED)
include_directories( ${Pangolin_INCLUDE_DIRS})

add_executable( undistort_image undistort_image.cpp)
target_link_libraries( undistort_image ${OpenCV_LIBS})

add_executable( disparity disparity.cpp)
target_link_libraries( disparity ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Pangolin_LIBRARIES})

add_executable( gaussnewton gaussnewton.cpp)
target_link_libraries( gaussnewton ${OpenCV_LIBS})