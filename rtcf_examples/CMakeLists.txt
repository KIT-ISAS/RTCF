cmake_minimum_required(VERSION 3.0.2)
project(rtcf_examples)

#############
## General ##
#############
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


##################
## Dependencies ##
##################
find_package(catkin REQUIRED COMPONENTS 
  rtt_rtcf_example_msgs
  rtcf)

catkin_package(CATKIN_DEPENDS 
  rtt_rtcf_example_msgs
  rtcf
)

#######################
## Actual components ##
#######################
add_subdirectory(src/bare_minimum)
add_subdirectory(src/identity)

# finalize the OROCOS package
orocos_generate_package()
