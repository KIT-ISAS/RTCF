cmake_minimum_required(VERSION 3.0.2)
project(rtcf_examples)

find_package(catkin REQUIRED COMPONENTS 
  rtt_ros
  rtt_rtcf_example_msgs
  rtcf)

catkin_package(CATKIN_DEPENDS 
  rtt_ros 
  rtt_rtcf_example_msgs
  rtcf
)

add_subdirectory(src/bare_minimum)
add_subdirectory(src/identity)

# finalize the OROCOS package
orocos_generate_package()
