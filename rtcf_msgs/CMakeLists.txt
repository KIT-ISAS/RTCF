cmake_minimum_required(VERSION 3.0.2)
project(rtcf_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Mapping.msg
  IterationInfo.msg
  # introspection
  PortInfo.msg
  ComponentInfo.msg
  ConnectionInfo.msg
  RunnerInfo.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  LoadOrocosComponent.srv
  UnloadOrocosComponent.srv
  GetRunnerInfo.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)