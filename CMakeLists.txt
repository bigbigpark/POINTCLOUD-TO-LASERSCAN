cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_2_laserscan)

add_compile_options(-std=c++17)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pointcloud_2_laserscan
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_2_laserscan src/pointcloud_2_laserscan.cpp)
target_link_libraries(pointcloud_2_laserscan
  ${catkin_LIBRARIES}
)