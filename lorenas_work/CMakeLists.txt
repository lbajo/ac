cmake_minimum_required(VERSION 2.8.3)
project(lorenas_work)


find_package(catkin REQUIRED COMPONENTS
  csuro_tools
  geometry_msgs
  pcl_ros
  roscpp
  rospy
  tf
  pcl_conversions
  image_transport
  sensor_msgs
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(LINK_LIBS
  ${catkin_LIBRARIES}
)

add_executable(nodo_vff
	src/nodo_vff.cpp
)

add_executable(nodo_def
	src/nodo_def.cpp
)

add_executable(nodo_scan
	src/nodo_scan.cpp
)

add_executable(nodo_rep 
	src/nodo_rep.cpp
)

add_executable(nodo_laser 
	src/nodo_laser.cpp
)

add_executable(follow_ball2 
	src/follow_ball.cpp
)

add_executable(nodo_hsv_tunner_2dmod
	src/nodo_hsv_tunner_2dmod.cpp
)

target_link_libraries(nodo_vff ${catkin_LIBRARIES})
target_link_libraries(nodo_def ${catkin_LIBRARIES})
target_link_libraries(nodo_scan ${catkin_LIBRARIES})
target_link_libraries(nodo_rep ${catkin_LIBRARIES})
target_link_libraries(nodo_laser ${catkin_LIBRARIES})
target_link_libraries(follow_ball2 ${catkin_LIBRARIES})
target_link_libraries(nodo_hsv_tunner_2dmod ${catkin_LIBRARIES})
