cmake_minimum_required(VERSION 3.0.2)
project(jh_localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  image_transport

  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

find_package(PCL REQUIRED QUIET)
# find_package(OpenCV REQUIRED QUIET)
find_package(Ceres REQUIRED)
find_package(OpenMP)
catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL EIGEN3
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jh_localization
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
	include
    src
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	# ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

link_directories(
	include
	# ${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
)

add_executable(localization src/localization.cpp src/lidarOptimization.cpp)
target_link_libraries(localization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES}   OpenMP::OpenMP_CXX) #${OpenCV_LIBS}
