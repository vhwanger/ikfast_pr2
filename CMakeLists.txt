cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

add_definitions(-DIKFAST_NO_MAIN=1)

#add_definitions(-DUSE_HARD_JOINT_LIMITS=1)
add_definitions(-DUSE_SOFT_JOINT_LIMITS=1)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/ikfast61.Transform6D.27_28_30_31_32_33_f29.cpp
                                     src/ikfast61.Transform6D.15_16_18_19_20_21_f17.cpp
                                     src/ik_interface.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
rosbuild_link_boost(${PROJECT_NAME} thread)

#rosbuild_add_executable(testarm src/armtest.cpp src/arm.cpp)
#target_link_libraries(testarm ${PROJECT_NAME})
#rosbuild_add_executable(testIK src/test.cpp)
#target_link_libraries(testIK ${PROJECT_NAME})
#rosbuild_add_executable(computeIK src/computeIK.cpp)
#target_link_libraries(computeIK ${PROJECT_NAME})
#
