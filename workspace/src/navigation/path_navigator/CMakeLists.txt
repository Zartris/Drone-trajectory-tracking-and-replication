cmake_minimum_required(VERSION 2.8.3)
project(path_navigator)
add_compile_options(-std=c++11)
set(BUILD_TYPE debug)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        tf
        roscpp
        std_msgs
        geometry_msgs
        martin_msg_lib
        bebop_msgs
        )


#add_message_files(
#        DIRECTORY
#        msg
#        FILES
#        Segment.msg
#)
#
#generate_messages(
#        DEPENDENCIES
#        std_msgs
#        geometry_msgs
#)

#add_message_files(
#        DIRECTORY msg
#        FILES Segment.msg)
#generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
        geometry_msgs
        bebop_msgs
        tf
)


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE
        )


###########
## Build ##
###########

set(CMAKE_CXX_FLAGS
        "${SSE_FLAGS} -O3 -g -std=c++0x -march=native"
        #   "${SSE_FLAGS} -O2 -g -std=c++0x -fno-omit-frame-pointer"
        )

catkin_package()
include_directories(
        include
        ${PROJECT_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
)
include_directories(${CSPARSE_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR})
## Declare a C++ library
#add_library(${PROJECT_NAME}
#        src/path_nav_node.cpp
#        )

#############
## Install ##
#############

add_executable(path_nav_node src/path_nav_node.cpp src/line.cpp src/vector3d.cpp include/path_navigator/bebop_command.h src/bebop_command.cpp src/MPC.cpp)
target_link_libraries(path_nav_node ${catkin_LIBRARIES} ipopt)
add_dependencies(path_nav_node martin_msg_lib_generate_messages_cpp)

#add_executable(line src/line.cpp src/vector3d.cpp)
#target_link_libraries(line ${catkin_LIBRARIES})
#
#add_executable(vector3d src/vector3d.cpp)
#target_link_libraries(line ${catkin_LIBRARIES})