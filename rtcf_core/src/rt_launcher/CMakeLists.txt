find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

add_executable(rt_launcher src/rt_launcher/rt_launcher_node.cpp)
add_dependencies(rt_launcher ${catkin_EXPORTED_TARGETS})

target_link_libraries(rt_launcher ${catkin_LIBRARIES})
