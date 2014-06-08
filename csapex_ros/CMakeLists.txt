cmake_minimum_required(VERSION 2.8.3)
project(csapex_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin COMPONENTS roscpp csapex csapex_core_plugins REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES csapex_ros_core
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(csapex_ros_core
    src/ros_handler.cpp
    src/ros_message_conversion.cpp
)


csapex_moc_all(QT_MOC src/*.h include/csapex_ros/*.h)

set(QT ${QT_SOURCES} ${QT_MOC})

## Declare a cpp library
add_library(csapex_ros
    src/apex_ros_interface.cpp
    src/export_ros.cpp
    src/import_ros.cpp
    ${QT_MOC}
)

## Specify libraries to link a library or executable target against
target_link_libraries(csapex_ros
  ${catkin_LIBRARIES}
)


#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS csapex_ros_core
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS csapex_ros
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
