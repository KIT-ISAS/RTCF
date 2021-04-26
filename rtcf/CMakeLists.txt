cmake_minimum_required(VERSION 3.0.2)
project(rtcf)


#############
## General ##
#############
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  dynamic_reconfigure
  rtcf_msgs
  rtt_ros 
  rtt_roscomm 
  rtt_rosclock
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rtcf_extension
  CATKIN_DEPENDS   
)

catkin_python_setup()

# OROCOS specific
find_package(OROCOS-RTT REQUIRED)
find_package(OROCOS-OCL REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
# needed for real-time logging in rtcf_extension
add_definitions(-DORO_MEMORY_POOL -DORO_BUILD_RTALLOC)
orocos_use_package(ocl-logging)

###########
## Build ##
###########

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src/rt_launcher)
add_subdirectory(src/rt_runner)
add_subdirectory(src/rtcf_extension)

#############
## Package ##
#############

# in addition to a catkin package, create an OROCOS package 
# (this will make OROCOS find the dependencies automatically by reading the package.xml)
orocos_generate_package(
  INCLUDE_DIRS include
)

catkin_install_python(PROGRAMS
  scripts/component_info
  scripts/component_order
  scripts/connection_info
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
