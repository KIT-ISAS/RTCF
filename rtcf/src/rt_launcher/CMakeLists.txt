find_package(Boost REQUIRED program_options)

add_executable(rt_launcher rt_launcher_node.cpp)
add_dependencies(rt_launcher ${catkin_EXPORTED_TARGETS})
target_link_libraries(rt_launcher ${catkin_LIBRARIES} ${Boost_LIBRARIES})
