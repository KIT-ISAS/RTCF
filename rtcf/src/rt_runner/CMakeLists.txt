# find OROCOS RTT
find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

# this will make the executable
add_executable(rt_runner 
  rt_runner_node.cpp
  rt_runner.cpp
  main_context.cpp)
# We could also use orocos_add_executable. However, this will add an ugly gnulinux-suffix
# So, we have to do some manual work 
# (see UseOROCOS-RTT.cmake and https://orocos.org/wiki/orocos/toolchain/getting-started/cmake-and-building.html)
orocos_add_include_directories(rt_runner ${OROCOS-RTT_INCLUDE_DIRS})
target_link_libraries(rt_runner ${OROCOS-RTT_LIBRARIES})

add_dependencies(rt_runner ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(rt_runner ${catkin_LIBRARIES})
