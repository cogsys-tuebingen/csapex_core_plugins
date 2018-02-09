cmake_minimum_required(VERSION 2.8.3)
project(csapex_unix)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    csapex
    
    )


catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES 
    CATKIN_DEPENDS
     
    DEPENDS system_lib
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    #include
    ${catkin_INCLUDE_DIRS}
    )

## Declare a C++ library
add_library(${PROJECT_NAME}_node
    src/subprocess_call.cpp
    src/bash_exec.cpp
    src/direct_exec.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    )

install(FILES plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


install(TARGETS ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
