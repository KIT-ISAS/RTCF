# this will make the executable
add_library(rtcf_extension 
  rtcf_extension.cpp  
  tlsf_memory_pool.cpp
  rt_rosconsole_logging.cpp
)
# We could also use orocos_add_executable. However, this will add an ugly gnulinux-suffix
# So, we have to do some manual work 
# (see UseOROCOS-RTT.cmake and https://orocos.org/wiki/orocos/toolchain/getting-started/cmake-and-building.html)
orocos_add_include_directories(rtcf_extension ${OROCOS-RTT_INCLUDE_DIRS} ${USE_OROCOS_INCLUDE_DIRS})
target_link_libraries(rtcf_extension ${OROCOS-RTT_LIBRARIES} ${USE_OROCOS_LIBRARIES})

add_dependencies(rtcf_extension ${catkin_EXPORTED_TARGETS})
target_link_libraries(rtcf_extension ${catkin_LIBRARIES})
