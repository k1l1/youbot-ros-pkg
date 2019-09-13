
cmake_minimum_required(VERSION 2.8.3)
project(youbot_battery_monitor)

find_package(catkin REQUIRED COMPONENTS roscpp pr2_msgs diagnostic_msgs)

catkin_package()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(youbot_battery_monitor src/youbot_battery_monitor.cpp nodes/youbot_battery_monitor_node.cpp )
target_link_libraries(youbot_battery_monitor stdc++ ${Boost_LIBRARIES})


