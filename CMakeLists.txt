cmake_minimum_required(VERSION 2.8.3)
project(people_tracker)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  roscpp
  rospy
  tf
  pcl_conversions
  cv_bridge
  image_transport
  sensor_msgs
  bica_core
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cob_people_tracking_filter
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES} 
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  $(cob_people_tracking_filter)
)

set(LINK_LIBS
  ${catkin_LIBRARIES}
  $(cob_people_tracking_filter)
)

add_executable(people_tracker_node 
	src/HSVHistogram.cpp
	src/Head.cpp
	src/Legs.cpp
	src/Body.cpp
	src/Slice.cpp
	src/InterestPointsScene.cpp
	src/Pblob.cpp
	src/PlobTracker.cpp
	src/PlobPerception.cpp
	src/FollowPeopleFSM.cpp
	src/people_tracker_node.cpp
)
target_link_libraries(people_tracker_node ${catkin_LIBRARIES})


add_executable(followpeople_test_node 
	src/followpeople_test_node.cpp
	src/FollowPeopleFSM.cpp
)

target_link_libraries(followpeople_test_node ${catkin_LIBRARIES})




#add_executable(people_test_siluete_perception
#	src/HSVHistogram.cpp
#	src/Head.cpp
#	src/Legs.cpp
#	src/Body.cpp
#	src/Slice.cpp
#	src/Pblob.cpp
#	src/InterestPointsScene.cpp
#	src/PlobPerception.cpp
#	src/TestMainPblobPerception.cpp
#)
#target_link_libraries(people_test_siluete_perception ${catkin_LIBRARIES})


#add_executable(people_test_points_scene 
#	src/HSVHistogram.cpp
#	src/Head.cpp
#	src/Legs.cpp
#	src/Body.cpp
#	src/Slice.cpp
#	src/Pblob.cpp
#	src/InterestPointsScene.cpp
#	src/PlobPerception.cpp
#	src/TestInterestPoints.cpp
#)
#target_link_libraries(people_test_points_scene  ${catkin_LIBRARIES})


#add_executable(cobPeopleActionNode 
#	src/cobPeopleActionNode.cpp
#)
#target_link_libraries(cobPeopleActionNode  ${catkin_LIBRARIES})
#add_dependencies(cobPeopleActionNode $(cob_people_tracking_filter))

#add_executable(actueNode 
#	src/actueNode.cpp
#)
#target_link_libraries(actueNode  ${catkin_LIBRARIES})


