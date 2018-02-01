cmake_minimum_required(VERSION 2.8.3)
project(ahrs_landmark)

## Add support for C++14, supported in ROS Kinetic and newer
 add_definitions(-std=c++14)

set(DEPS
  message_filters
  roscpp
  sensor_msgs
  std_msgs
  tf
  vbus_sockets
)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${DEPS}
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
    AhrsLmrk.msg
)

generate_messages(DEPENDENCIES 
  sensor_msgs
  std_msgs
)



###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ahrs_landmark
  CATKIN_DEPENDS message_runtime  ${DEPS}
)

###########
## Build ##
###########

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}
     src/serial/SerialLink.cpp
     src/ahrsLandmark/SerialAhrs.cpp
     src/ahrsLandmark/AhrsFrame.cpp
     src/ahrsLandmark/AhrsLandmark.cpp
     src/ahrsLandmark/AhrsLandmarkNode.cpp
 )

## Add cmake target dependencies of the executable

add_dependencies(${PROJECT_NAME}
   ${PROJECT_NAME}_generate_messages_cpp 
   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
   ${catkin_EXPORTED_TARGETS}
 )

## Specify libraries to link a library or executable target against

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )

#############
## Install ##
#############

## Mark cpp header files for installation

 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

install(DIRECTORY
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
