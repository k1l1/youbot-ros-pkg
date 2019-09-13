cmake_minimum_required(VERSION 2.8.3)
project(hanoi)

catkin_package()

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs brics_actuator trajectory_msgs tf sensor_msgs arm_navigation_msgs)

include_directories(${catkin_INCLUDE_DIRS}
					${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(
	include	
	${INCLUDE_BOOST_DIRS}
	${catkin_LIBARARIES}
)

add_definitions(${EIGEN_DEFINITIONS})

add_executable(hanoi src/main.cpp src/armPosition.cpp src/CArmControl.cpp src/CKinematics.cpp)
add_executable(armPath src/armPath.cpp src/armPosition.cpp src/CArmControl.cpp src/CKinematics.cpp)
add_executable(gripper src/gripper.cpp src/armPosition.cpp)
add_executable(wrist src/wrist.cpp src/armPosition.cpp)

target_link_libraries(hanoi
		${catkin_LIBRARIES})
target_link_libraries(armPath
		${catkin_LIBRARIES})
target_link_libraries(gripper
		${catkin_LIBRARIES})
target_link_libraries(wrist
		${catkin_LIBRARIES})
