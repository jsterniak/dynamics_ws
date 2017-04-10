cmake_minimum_required(VERSION 2.8.3)
project(irb120_vision)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
)

find_package(OpenCV 2.4 REQUIRED core gpu imgproc calib3d highgui)

## Generate added messages and services
#generate_messages(DEPENDENCIES
#  sensor_msgs
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES irb120_vision
  CATKIN_DEPENDS cv_bridge roscpp sensor_msgs
#  DEPENDS system_lib
)

#recurse for headers so they are displayed
file (GLOB_RECURSE HEADERS
*.h
*.hpp
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#all c/cpp files and headers above, except file with main
set (SOURCE_FILES_LIB
  ${HEADERS}
  src/irb120_vision.cpp
)

#source file with main() only
set (SOURCE_FILES
  src/irb120_vision_node.cpp
)

add_library(${PROJECT_NAME}_lib ${SOURCE_FILES_LIB})
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib ${catkin_LIBRARIES})
