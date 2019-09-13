cmake_minimum_required(VERSION 2.8.3)
project(simple_trajectory_controller)

find_package(catkin REQUIRED COMPONENTS planning_environment arm_kinematics_constraint_aware ompl_ros_interface trajectory_filter_server constraint_aware_spline_smoother move_arm trajectory_msgs brics_actuator)

catkin_package()

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

add_executable(youbot_trajectory_controller src/youbot_trajectory_controller.cpp)

