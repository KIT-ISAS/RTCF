cmake_minimum_required(VERSION 2.8.3)
project(rtt_rtcf_example_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

ros_generate_rtt_typekit(rtcf_example_msgs)
ros_generate_rtt_service_proxies(rtcf_example_msgs)

orocos_generate_package(
  DEPENDS rtcf_example_msgs
  DEPENDS_TARGETS rtt_roscomm  rtt_std_msgs
 rtt_geometry_msgs

)
