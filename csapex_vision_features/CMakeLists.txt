cmake_minimum_required(VERSION 2.8.3)
project(csapex_vision_features)

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
find_package(catkin REQUIRED COMPONENTS csapex csapex_vision utils_vision)

# note to ubuntu 14 users: if opencv is installed at /usr/local and is not found:    sudo ldconfig /usr/local/lib
set(USED_OpenCV_COMPONENTS core imgproc ml calib3d)
find_package(OpenCV COMPONENTS nonfree ${USED_OpenCV_COMPONENTS} QUIET CONFIG)

list(FIND OpenCV_LIB_COMPONENTS opencv_nonfree NONFREE_INDEX)
set(NONFREE_FOUND NOT ${NONFREE_INDEX} EQUAL -1)

if(${OpenCV_FOUND} AND ${NONFREE_FOUND})
    add_definitions("-DCV_NON_FREE=1")
else()
    add_definitions("-DCV_NON_FREE=0")
    message("OpenCV nonfree was not found, SIFT and SURF will not be available. If you need them, compile OpenCV from source and remove CMakeCache.txt!")
    find_package(OpenCV REQUIRED COMPONENTS ${USED_OpenCV_COMPONENTS} CONFIG)
endif()

find_package(Boost COMPONENTS program_options REQUIRED)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)

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
  LIBRARIES csapex_vision_features
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
)

add_library(csapex_vision_features
    src/keypoint_message.cpp
    src/descriptor_message.cpp
    src/match_message.cpp

    src/extractors_opencv.cpp
    src/extractor_factory.cpp
    src/extractor_manager.cpp
)
target_link_libraries(csapex_vision_features
    yaml-cpp ${QT_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(csapex_vision_features_plugin_core
    src/register_plugin.cpp
)
target_link_libraries(csapex_vision_features_plugin_core csapex_vision_features
    yaml-cpp ${QT_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${catkin_LIBRARIES})


add_library(csapex_vision_features_plugin_node
    src/extract_keypoints.cpp
    src/extract_descriptors.cpp
    src/grid_arranged_keypoints.cpp
    src/find_homography.cpp
    src/display_keypoints.cpp
    src/match_descriptors.cpp
)
target_link_libraries(csapex_vision_features_plugin_node csapex_vision_features
    yaml-cpp ${QT_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${catkin_LIBRARIES})


#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS csapex_vision_features
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_vision_features_plugin_core
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS csapex_vision_features_plugin_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

