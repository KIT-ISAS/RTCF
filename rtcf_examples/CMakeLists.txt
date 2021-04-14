cmake_minimum_required(VERSION 3.0.2)
project(rtcf_examples)

#############
## General ##
#############
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
orocos_use_package(ocl-logging)

##################
## Dependencies ##
##################
find_package(catkin REQUIRED COMPONENTS 
  rtt_rtcf_example_msgs
  rtcf
  dynamic_reconfigure)

#######################
## Actual components ##
#######################
add_subdirectory(src/bare_minimum)
add_subdirectory(src/identity)
add_subdirectory(src/mimo)
add_subdirectory(src/parameter_handling)

# finalize the OROCOS package
orocos_generate_package()

catkin_package()