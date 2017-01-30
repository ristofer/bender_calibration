cmake_minimum_required(VERSION 2.8.3)
project(bender_calibration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    actionlib
    camera_calibration_parsers
    cv_bridge
    geometry_msgs
    kdl_parser
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
    camera_calibration_parsers
    cv_bridge
    geometry_msgs
    kdl_parser
    moveit_msgs
    nav_msgs
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

target_link_libraries(chain_test ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(feature_test src/checkerboard_finder.cpp
                            src/feature_finder.cpp
                            src/feature_test.cpp)

<<<<<<< HEAD
#add_library(robot_calibration src/chain_manager.cpp)

target_link_libraries(feature_test ${Boost_LIBRARIES} ${catkin_LIBRARIES})
=======
target_link_libraries(feature_test ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(calibration_test src/calibration_test.cpp
                                src/chain_manager.cpp
                                src/feature_finder.cpp
                                src/checkerboard_finder.cpp)

target_link_libraries(calibration_test ${Boost_LIBRARIES} ${catkin_LIBRARIES})

>>>>>>> 8da5bfd2e95478da8d64bec1746111ee30a13a2a
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
<<<<<<< HEAD
)
=======
)
>>>>>>> 8da5bfd2e95478da8d64bec1746111ee30a13a2a
