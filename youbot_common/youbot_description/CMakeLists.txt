# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(youbot_description)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS robot_state_publisher urdf trajectory_msgs brics_actuator nav_msgs pr2_controller_interface control_toolbox pr2_controllers_msgs realtime_tools orocos_kdl sensor_msgs control_msgs actionlib youbot_trajectory_action_server pr2_gazebo_plugins)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(	${PROJECT_SOURCE_DIR}/include
			${PROJECT_SOURCE_DIR}/src)

#uncomment if you have defined messages
#add_message_files(
  FILES
  # TODO: List your msg files here
)
#uncomment if you have defined services
#add_service_files(
  FILES
  # TODO: List your msg files here
)

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

OPTION(WITH_DEBUG_INFO "Publish debug info" ON)	#TODO:: move debug parameter to the yaml file

IF(WITH_DEBUG_INFO)
  ADD_DEFINITIONS(-DDEBUG_INFO)
ENDIF(WITH_DEBUG_INFO)

#youbot base teleoperation experiment 
add_executable(youbot_keyboard_teleop_experiment examples/keyboard_teleop_experiment.cpp)

#test for a joint position control of the youbot arm
add_library(joint_position_control src/joint_position_control.cpp)

#youbot universal controller: joint space position control, joint velocity control, joint torque control
add_library(youbot_universal_control src/youbot_universal_control.cpp src/joint_state_observer_gazebo.cpp)
target_link_libraries(youbot_universal_control youbot_trajectory_action_server)
add_executable(arm_joint_position_control_test examples/arm_joint_position_control_test.cpp)
add_executable(arm_joint_velocity_control_test examples/arm_joint_velocity_control_test.cpp)
add_executable(arm_joint_torque_control_test examples/arm_joint_torque_control_test.cpp)

#cartesian compliance control for the unified youbot arm and base system
#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/20_sim_cartesian_compliance_control)
#add_library(cartesian_compliance_control src/cartesian_compliance_control.cpp)
#target_link_libraries(cartesian_compliance_control 20_sim_cartesian_compliance_control)
#add_executable(cartesian_compliance_control_test examples/cartesian_compliance_control_test.cpp)

#static joint states publisher
add_executable(static_joint_publisher src/static_joint_publisher.cpp)

#depricated
add_executable(base_command_republisher src/base_command_republisher.cpp)

## Generate added messages and services with any dependencies listed here
generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS robot_state_publisher urdf trajectory_msgs brics_actuator nav_msgs pr2_controller_interface control_toolbox pr2_controllers_msgs realtime_tools orocos_kdl sensor_msgs control_msgs actionlib youbot_trajectory_action_server pr2_gazebo_plugins
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)