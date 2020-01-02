cmake_minimum_required(VERSION 3.8.2)
project(csapex_vision_features CXX)

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
find_package(catkin REQUIRED COMPONENTS csapex csapex_opencv cslibs_vision)

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
  CATKIN_DEPENDS csapex_opencv
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

add_library(${PROJECT_NAME}
    src/keypoint_message.cpp
    src/descriptor_message.cpp
    src/match_message.cpp

    src/extractors_opencv.cpp
    src/extractor_factory.cpp
    src/extractor_manager.cpp
)
target_link_libraries(${PROJECT_NAME}
    yaml-cpp ${QT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_plugin_core
    src/register_plugin.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin_core ${PROJECT_NAME}
    yaml-cpp ${QT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})


add_library(${PROJECT_NAME}_plugin_node
    src/extract_keypoints.cpp
    src/extract_descriptors.cpp
    src/grid_arranged_keypoints.cpp
    src/find_homography.cpp
    src/display_keypoints.cpp
    src/match_descriptors.cpp    
    src/lk_tracking.cpp
)
target_link_libraries(csapex_vision_features_plugin_node csapex_vision_features
    yaml-cpp ${QT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})


#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugin_core  ${PROJECT_NAME}_plugin_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

