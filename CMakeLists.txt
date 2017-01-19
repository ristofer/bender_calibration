cmake_minimum_required(VERSION 2.8.3)
project(bender_calibration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    actionlib
    geometry_msgs
    moveit_msgs
    nav_msgs
    robot_calibration_msgs
    rosbag
    roscpp
    sensor_msgs
    std_msgs
    tf
)

## System dependencies are found with CMake's conventions

find_package(Boost REQUIRED COMPONENTS system signals thread)


catkin_python_setup()




catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    moveit_msgs
    nav_msgs
    robot_calibration
    robot_calibration_msgs
    rosbag
    roscpp
    sensor_msgs
    std_msgs
    tf
  DEPENDS
    Boost
  LIBRARIES
    robot_calibration
)


include_directories(include ${Boost_INCLUDE_DIRS}
                            ${catkin_INCLUDE_DIRS})
 # this is a hack, will eventually be unneeded once orocos-kdl is fixed

add_executable(chain_test src/chain_test.cpp
                          src/chain_manager.cpp)
#add_library(robot_calibration src/chain_manager.cpp)
target_link_libraries(chain_test ${Boost_LIBRARIES} ${catkin_LIBRARIES})
## - - - RecognizerROS- - - 
# add_executable(recognizer_ros
#   src/RecognizerROS.cpp
#   src/Recognizer.cpp
#   src/AudioSource.cpp
#   src/Exceptions.cpp
# )
# add_dependencies(recognizer_ros ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(recognizer_ros ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PocketSphinx_LIBRARIES} ${SphinxBase_LIBRARIES} ${SphinxAD_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
