find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

add_executable(rt_dummy src/rt_dummy/rt_dummy_node.cpp)

target_link_libraries(rt_dummy ${catkin_LIBRARIES})
