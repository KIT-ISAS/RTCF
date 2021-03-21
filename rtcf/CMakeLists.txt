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
  std_msgs 
  message_generation 
  rtt_ros 
  rtt_roscomm 
  rtt_rosclock
)

##############
## Messages ##
##############

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Mapping.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   LoadOrocosComponent.srv
   UnloadOrocosComponent.srv
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

 
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS   
  message_runtime 
  rtt_ros 
  rtt_roscomm 
  rtt_rosclock
)

###########
## Build ##
###########

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src/rt_launcher)
add_subdirectory(src/rt_runner)

# in addition to a catkin package, create an OROCOS package 
# (this will make OROCOS find the dependencies automatically by reading the package.xml)
orocos_generate_package(
  INCLUDE_DIRS include
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
