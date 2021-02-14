# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#


# Creates a commponent executable
# and installs in the directory lib/orocos/slave_activity_non_rt/
#
orocos_executable(rt_runner src/rt_runner/rt_runner_node.cpp
    src/rt_runner/rt_runner.cpp
    src/rt_runner/main_context.cpp)

target_link_libraries(rt_runner ${catkin_LIBRARIES})

orocos_install_headers(src/rt_runner/rt_runner_node.hpp include/rtcf/rtcf_extension.hpp) # ...you may add multiple header files

#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package(
  INCLUDE_DIRS include
    )
