cmake_minimum_required(VERSION 3.0.2)
project(hector_stairs_planner_msgs VERSION 0.0.0)

find_package(catkin REQUIRED COMPONENTS message_generation nav_msgs std_msgs)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Path_segment.msg
   Path_with_Flipper.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   StairDectection_StairsCloud_Srv.srv
#   StairDetection_StairPosition_Srv.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   nav_msgs
 )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS message_runtime nav_msgs std_msgs
)
