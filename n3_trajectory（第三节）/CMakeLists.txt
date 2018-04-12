cmake_minimum_required(VERSION 2.8)
project(draw_trajectory)
add_definitions("-std=c++11")

include_directories("/usr/include/eigen3")

find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
include_directories(
        ${Sophus_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
)


add_executable(draw_trajectory draw_trajectory.cpp)
target_link_libraries(draw_trajectory ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES})

add_executable(comp_t comp_t.cpp)
target_link_libraries(comp_t ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES})