cmake_minimum_required(VERSION 2.8.3)
project(testing)
find_package(catkin REQUIRED tf2_ros tf2_geometry_msgs pcl_conversions pcl_ros sensor_msgs cv_bridge roscpp std_msgs image_transport)
find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
		INCLUDE_DIRS include
		LIBRARIES depthmap2cloud
		CATKIN_DEPENDS tf2_ros tf2_geometry_msgs pcl_conversions pcl_ros sensor_msgs cv_bridge roscpp std_msgs image_transport)
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(depthmap2cloud src/depthmap2cloud.cpp)
target_link_libraries(depthmap2cloud ${catkin_LIBRARIES})
target_link_libraries(depthmap2cloud
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
)

install(TARGETS depthmap2cloud
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		)
