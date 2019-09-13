# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(youbot_oodl)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS std_msgs roscpp tf geometry_msgs nav_msgs std_srvs trajectory_msgs brics_actuator youbot_driver control_msgs actionlib diagnostic_msgs pr2_msgs setcap)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

## check required prerequisites
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/cmake_modules")
SET(Boost_ADDITIONAL_VERSIONS "1.39" "1.39.0" "1.40" "1.40.0" "1.41" "1.41.0" "1.42" "1.42.0" "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46" "1.46.0")
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread date_time filesystem system)


rosbuild_find_ros_package(youbot_driver)

## preprocessor definitions
ADD_DEFINITIONS(-DYOUBOT_CONFIGURATIONS_DIR="${youbot_driver_PACKAGE_PATH}/config/")


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

## Set Compilter flags
INCLUDE_DIRECTORIES(
	${OODL_YOUBOT_INCLUDE_DIR}
	${Boost_INCLUDE_DIR}
)	


add_executable(youbot_oodl 
    src/youbot_oodl.cpp 
    src/YouBotOODLWrapper.cpp
    src/YouBotConfiguration.cpp
    src/joint_state_observer_oodl.cpp
)

target_link_libraries(youbot_oodl ${OODL_YOUBOT_LIBRARIES} ${Boost_LIBRARIES})

OPTION(USE_SETCAP "Set permissions to access ethernet interface without sudo" ON)

SET(SUDO_COMMAND gksudo)
IF($ENV{USE_NORMAL_SUDO})
    SET(SUDO_COMMAND sudo)
ENDIF($ENV{USE_NORMAL_SUDO})

IF(USE_SETCAP)
add_custom_command(TARGET youbot_oodl POST_BUILD
    #COMMAND sudo setcap cap_net_raw+ep youbot_oodl
    COMMAND ${SUDO_COMMAND} setcap cap_net_raw+ep youbot_oodl
    WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}
)    
endif(USE_SETCAP)

add_executable(youbot_arm_test 
    examples/youbot_arm_test.cpp 
)

add_executable(youbot_2nd_arm_test 
    examples/youbot_2nd_arm_test.cpp 
)

#add_executable(youbot_arm_velocity_test 
#    examples/youbot_arm_velocity_test.cpp 
#)
## Generate added messages and services with any dependencies listed here
generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS std_msgs roscpp tf geometry_msgs nav_msgs std_srvs trajectory_msgs brics_actuator youbot_driver control_msgs actionlib diagnostic_msgs pr2_msgs setcap
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)