find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

add_message_files(
  DIRECTORY msg
  FILES
  LaneInfo.msg
  ObstacleList.msg
  ObstacleInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)