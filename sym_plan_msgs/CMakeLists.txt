cmake_minimum_required(VERSION 3.0.2)
project(sym_plan_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
  catkin
  REQUIRED
  COMPONENTS roscpp
             rospy
             std_msgs
             moveit_msgs
             message_generation
)

add_service_files(
  FILES
  ProcessAttachedCollisionObject.srv
  ProcessCollisionObject.srv
  RequestIK.srv
  RequestPlan.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  moveit_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)
