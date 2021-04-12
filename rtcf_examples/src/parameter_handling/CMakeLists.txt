# add dynamic reconfigure example config
# (this needs to go here as it must appear before catkin_package())
generate_dynamic_reconfigure_options(
  cfg/Example.cfg
)

orocos_component(parameter_handling component.cpp)
target_link_libraries(parameter_handling ${catkin_LIBRARIES})
add_dependencies(parameter_handling ${rtcf_examples_EXPORTED_TARGETS})
