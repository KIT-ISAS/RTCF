
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

# Rest of the configuration is in src/
#add_subdirectory( rtcf/src/rt_runner/src/CMakeLists.txt )

#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

orocos_use_package(ocl-taskbrowser)

# Creates a commponent executable
# and installs in the directory lib/orocos/slave_activity_non_rt/
#
orocos_executable(rt_runner src/rt_runner/rt_runner_node.cpp
    src/rt_runner/rt_runner.cpp
    src/rt_runner/main_context.cpp)
#
# You may add multiple orocos_component statements.

target_link_libraries(rt_runner ${catkin_LIBRARIES})

#
# Additional headers:
#
# Installs in the include/orocos/slave_activity_non_rt/ directory
#
orocos_install_headers(src/rt_runner/rt_runner_node.hpp) # ...you may add multiple header files

#
# You may add multiple orocos_install_headers statements.


#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
