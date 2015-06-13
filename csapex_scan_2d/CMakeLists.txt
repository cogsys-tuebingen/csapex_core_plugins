cmake_minimum_required(VERSION 2.8.3)
project(csapex_scan_2d)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS csapex csapex_ros csapex_vision utils_laser_processing)

find_package(Boost REQUIRED)

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
  LIBRARIES csapex_scan_2d
  CATKIN_DEPENDS utils_laser_processing
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(csapex_scan_2d
    src/scan_message.cpp
    src/labeled_scan_message.cpp
)
target_link_libraries(csapex_scan_2d
    yaml-cpp ${Boost_LIBRARIES} ${catkin_LIBRARIES})


add_library(csapex_scan_2d_plugin
    src/register_plugin.cpp

    src/segment_labeler.cpp
    src/split_labeled_scan.cpp
    src/labeled_scan_to_scan.cpp
    src/renderer.cpp
    src/segmentation_renderer.cpp
    src/segment_distance_filter.cpp
    src/segment_mincount_filter.cpp
    src/segment_label_filter.cpp
    src/scan_renderer.cpp
    src/scan_message_renderer.cpp
    src/scan_labeler.cpp
    src/scan_labeler_adapter.cpp
)

target_link_libraries(csapex_scan_2d_plugin
    yaml-cpp csapex_scan_2d ${catkin_LIBRARIES})


#add_executable(scan_test src/test.cpp)
#target_link_libraries(scan_test csapex_scan_2d_plugin csapex_scan_2d)

#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS csapex_scan_2d
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

