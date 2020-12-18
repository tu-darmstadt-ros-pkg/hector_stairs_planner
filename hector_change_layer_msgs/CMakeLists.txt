cmake_minimum_required(VERSION 3.0.2)
project(hector_change_layer_msgs VERSION 0.0.0)

find_package(catkin REQUIRED COMPONENTS
message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

 add_message_files(
   FILES
   Change_layer_msg.msg
   MapLayerList.msg
 )

 generate_messages(
   DEPENDENCIES
 )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS message_runtime
)

