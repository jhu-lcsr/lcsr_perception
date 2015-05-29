cmake_minimum_required(VERSION 2.8.3)
project(stacked_stereo_transport)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport message_filters cv_bridge)

find_package(OpenCV REQUIRED)

include_directories(include)

catkin_package(
  INCLUDE_DIRS include
)

## Declare a cpp executable
add_library(stereo_stacking 
  src/stereo_stacker.cpp
  src/stereo_unstacker.cpp)
target_link_libraries(
  stereo_stacking
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(stereo_stacker src/stereo_stacker_node.cpp)
target_link_libraries(stereo_stacker stereo_stacking)

add_executable(stereo_unstacker src/stereo_unstacker_node.cpp)
target_link_libraries(stereo_unstacker stereo_stacking)
