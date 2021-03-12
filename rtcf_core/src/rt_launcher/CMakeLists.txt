find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
find_package(Boost REQUIRED program_options)

add_executable(rt_launcher src/rt_launcher/rt_launcher_node.cpp)
add_dependencies(rt_launcher ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(rt_launcher ${catkin_LIBRARIES} ${Boost_LIBRARIES})
