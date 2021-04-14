# this will make the executable
orocos_library(rtcf_extension 
  rtcf_extension.cpp  
  tlsf_memory_pool.cpp
  rt_rosconsole_logging.cpp
)
add_dependencies(rtcf_extension ${catkin_EXPORTED_TARGETS})
target_link_libraries(rtcf_extension ${catkin_LIBRARIES})
