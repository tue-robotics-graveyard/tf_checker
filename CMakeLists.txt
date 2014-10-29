cmake_minimum_required(VERSION 2.8.3)
project(tf_checker)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

catkin_package(
	#  INCLUDE_DIRS include
	#  LIBRARIES depth_map_to_point_cloud
	#  CATKIN_DEPENDS cv_bridge image_geometry pcl_ros roscpp sensor_msgs stereo_msgs tf
	#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_checker src/tf_checker.cpp)

target_link_libraries(tf_checker ${catkin_LIBRARIES})
