# this will make the executable
add_executable(rt_runner 
  rt_runner_node.cpp
  rt_runner.cpp
  main_context.cpp
  timing_analysis.cpp)
# We could also use orocos_add_executable. However, this will add an ugly gnulinux-suffix
# So, we have to do some manual work 
# (see UseOROCOS-RTT.cmake and https://orocos.org/wiki/orocos/toolchain/getting-started/cmake-and-building.html)
orocos_add_include_directories(rt_runner ${OROCOS-RTT_INCLUDE_DIRS} ${USE_OROCOS_INCLUDE_DIRS})
target_link_libraries(rt_runner ${OROCOS-RTT_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_compile_definitions(rt_runner PUBLIC -DRTT_STATIC)

add_dependencies(rt_runner ${catkin_EXPORTED_TARGETS})
target_link_libraries(rt_runner
  ${catkin_LIBRARIES}
  rtcf_extension
)
