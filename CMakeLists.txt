cmake_minimum_required(VERSION 2.8.3)
project(mapping)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  mavros
  mavros_msgs
  roscpp
  rospy
  std_msgs
  cv_bridge
)

find_package (GeographicLib 1.34 REQUIRED)

find_package (OpenCV)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   mavros_msgs
   std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mapping
   CATKIN_DEPENDS mavros mavros_msgs roscpp rospy std_msgs cv_bridge
#  DEPENDS system_lib
)

add_executable(mapping src/mavrosCommandMain.cpp src/mavrosCommand.cpp)
target_link_libraries(mapping ${catkin_LIBRARIES} ${OpenCV_LIBS})


