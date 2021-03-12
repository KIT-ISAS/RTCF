cmake_minimum_required(VERSION 3.0.2)
project(rtcf_examples)

# find OROCOS RTT
find_package(OROCOS-RTT REQUIRED)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

add_subdirectory(src/bare_minimum)

# finalize the OROCOS package
orocos_generate_package()
