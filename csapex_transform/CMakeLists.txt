cmake_minimum_required(VERSION 2.8.3)
project(csapex_transform)

ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

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
find_package(catkin REQUIRED COMPONENTS csapex csapex_core_plugins csapex_ros tf)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES csapex_transform
#  CATKIN_DEPENDS csapex csapex_core_plugins
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(csapex_transform
    src/transform_message.cpp
)
target_link_libraries(csapex_transform
    yaml-cpp ${QT_LIBRARIES} ${catkin_LIBRARIES})


add_library(csapex_transform_plugin_core
    src/register_plugin.cpp
)
target_link_libraries(csapex_transform_plugin_core csapex_transform
    yaml-cpp ${QT_LIBRARIES} ${catkin_LIBRARIES})




add_library(csapex_transform_plugin_node
    src/extract_frames.cpp
    src/transform_to_odometry.cpp
    src/transform_to_pose.cpp
    src/odometry_to_transform.cpp
    src/pose_to_transform.cpp
    src/clock.cpp
    src/time_offset.cpp
    src/static_transform.cpp
    src/dynamic_transform.cpp
    src/transform_combiner.cpp
    src/transform_inverter.cpp
    src/transform_publisher.cpp
    src/transform_filter.cpp
    src/extract_timestamp.cpp
    src/extract_measures.cpp
)
target_link_libraries(csapex_transform_plugin_node csapex_transform
    yaml-cpp ${QT_LIBRARIES} ${catkin_LIBRARIES})


#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS csapex_transform
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_transform_plugin_core
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS csapex_transform_plugin_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

