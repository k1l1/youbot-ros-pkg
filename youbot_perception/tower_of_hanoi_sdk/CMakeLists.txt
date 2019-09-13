# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tower_of_hanoi_sdk)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS tf std_msgs roscpp rospy sensor_msgs geometry_msgs arm_navigation_msgs visualization_msgs image_transport pcl pcl_ros opencv2 cv_bridge dynamic_reconfigure brics_3d)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)



# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#Set the path for external CMAKE modules
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/cmake_modules")


# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()


add_message_files(
  FILES
  # TODO: List your msg files here
)
add_service_files(
  FILES
  # TODO: List your msg files here
)

ADD_DEFINITIONS(-DEIGEN3)

# add include directories (-I)
INCLUDE_DIRECTORIES(
     ${PROJECT_SOURCE_DIR}/lib
)




#ADD_SUBDIRECTORY(./lib) #compile everything in lib to create the libraries

#executable for color based region of interest extraction
add_executable(colorBasedRoiExtractor src/ColorBasedRoiExtractionNode.cpp 
						src/examples/ColorBasedRoiExtractor)
target_link_libraries(colorBasedRoiExtractor brics3d_util.a brics3d_algorithm brics3d_core)

#executable for hsv configuration finder
add_executable(hsvLimitsFinder src/HSVLimitsFinderNode.cpp 
						src/examples/ColorBasedRoiExtractor)
target_link_libraries(hsvLimitsFinder  brics3d_util brics3d_algorithm brics3d_core)

#executable for object cluster extraction
add_executable(objectClusterExtractor src/ObjectClusterExtractionNode.cpp 
						src/examples/EuclideanClusterExtractor lib/EuclideanClustering)
target_link_libraries(objectClusterExtractor   
										brics3d_util brics3d_algorithm brics3d_core)

#executable for Model Fitting using ICP
add_executable(modelFittingICP src/ModelFittingICPNode.cpp 
						src/examples/ModelFitting lib/IterativeClosestPoint)
target_link_libraries(modelFittingICP   
										brics3d_util brics3d_algorithm brics3d_core)										

										
#executable for 6D Pose Estimation
add_executable(poseEstimator6D src/PoseEstimation6DNode.cpp 
						src/examples/PoseEstimation6D lib/IterativeClosestPoint lib/EuclideanClustering)
target_link_libraries(poseEstimator6D   
										brics3d_util brics3d_algorithm brics3d_core)
										
#executable for controlling the perception system
add_executable(perceptionEngineController src/ControlPerception.cpp)

#executable for configuring the perception system
add_executable(perceptionEngineConfigurator src/ConfigurePerception.cpp)

#executable for 3D world model
add_executable(worldModel src/WorldModelNode.cpp )
target_link_libraries(worldModel brics3d_world_model brics3d_util brics3d_algorithm brics3d_core)

#examples for 3D world model
add_executable(worldModelSampleQuery src/examples/WorldModelSampleQuery.cpp )
add_executable(worldModelVisualizer src/examples/WorldModelVisualizer.cpp )


## Generate added messages and services with any dependencies listed here
generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS tf std_msgs roscpp rospy sensor_msgs geometry_msgs arm_navigation_msgs visualization_msgs image_transport pcl pcl_ros opencv2 cv_bridge dynamic_reconfigure brics_3d
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)