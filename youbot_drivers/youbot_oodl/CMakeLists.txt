cmake_minimum_required(VERSION 2.8.3)
project(youbot_oodl)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS std_msgs roscpp tf geometry_msgs nav_msgs std_srvs trajectory_msgs brics_actuator youbot_driver control_msgs actionlib diagnostic_msgs pr2_msgs ) #setcap)

catkin_package()

## check required prerequisites
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/cmake_modules")
set(Boost_ADDITIONAL_VERSIONS "1.39" "1.39.0" "1.40" "1.40.0" "1.41" "1.41.0" "1.42" "1.42.0" "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46" "1.46.0")
find_package(Boost REQUIRED COMPONENTS thread date_time filesystem system)

## preprocessor definitions
add_definitions(-DYOUBOT_CONFIGURATIONS_DIR=/usr/local/config)


set(OODL_YOUBOT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

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
include_directories(${Boost_INCLUDE_DIRS}
	            ${catkin_INCLUDE_DIRS}
		    ${OODL_YOUBOT_INCLUDE_DIR}
		    #bugfix (using absolute path to folder)
		    /usr/local/include/youbot_driver/)

add_executable(youbot_oodl 
    src/youbot_oodl.cpp 
    src/YouBotOODLWrapper.cpp
    src/YouBotConfiguration.cpp
    src/joint_state_observer_oodl.cpp
)

get_cmake_property(_variableNames VARIABLES)
list (SORT _variableNames)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()

target_link_libraries(youbot_oodl ${OODL_YOUBOT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

#option(USE_SETCAP "Set permissions to access ethernet interface without sudo" ON)

#SET(SUDO_COMMAND gksudo)
#IF($ENV{USE_NORMAL_SUDO})
#    SET(SUDO_COMMAND sudo)
#ENDIF($ENV{USE_NORMAL_SUDO})
#
#IF(USE_SETCAP)
#add_custom_command(TARGET youbot_oodl POST_BUILD
    #COMMAND sudo setcap cap_net_raw+ep youbot_oodl
    #COMMAND ${SUDO_COMMAND} setcap cap_net_raw+ep youbot_oodl
    #WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}
#)    
#endif(USE_SETCAP)

add_executable(youbot_arm_test 
    examples/youbot_arm_test.cpp 
)

target_link_libraries(youbot_arm_test ${OODL_YOUBOT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(youbot_2nd_arm_test 
    examples/youbot_2nd_arm_test.cpp 
)

target_link_libraries(youbot_2nd_arm_test ${OODL_YOUBOT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(youbot_arm_velocity_test 
#    examples/youbot_arm_velocity_test.cpp 
#)
