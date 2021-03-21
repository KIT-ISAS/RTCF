find_package(rostest REQUIRED)

# sub directories
add_subdirectory(rt_launcher)
add_subdirectory(rt_runner)
add_subdirectory(component_sum)
add_subdirectory(component_extension)

# integration tests
add_rostest_gtest(
    test_integration_sum 
    launch/test_integration_sum.test
    test_integration_sum.cpp)
target_link_libraries(test_integration_sum ${catkin_LIBRARIES})

add_rostest_gtest(
    test_integration_extension 
    launch/test_integration_extension.test
    test_integration_extension.cpp)
target_link_libraries(test_integration_extension ${catkin_LIBRARIES})