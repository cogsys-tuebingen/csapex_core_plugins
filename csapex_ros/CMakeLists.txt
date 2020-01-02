cmake_minimum_required(VERSION 3.8.2)
project(csapex_ros CXX)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin COMPONENTS csapex csapex_core_plugins  roscpp rosbag topic_tools tf pluginlib REQUIRED)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_core
  CATKIN_DEPENDS pluginlib csapex_core_plugins
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ${Qt5Core_INCLUDE_DIRS} 
  ${Qt5Widgets_INCLUDE_DIRS} 
  ${Qt5Gui_INCLUDE_DIRS}
)

## booting
add_library(${PROJECT_NAME}_boot
    src/ros_boot.cpp
)
target_link_libraries(${PROJECT_NAME}_boot
  ${catkin_LIBRARIES}
)

add_custom_command(TARGET ${PROJECT_NAME}_boot POST_BUILD
    COMMAND mkdir -p ${CSAPEX_BOOT_PLUGIN_DIR} && ln -sf ${CATKIN_DEVEL_PREFIX}/lib/lib${PROJECT_NAME}_boot.so ${CSAPEX_BOOT_PLUGIN_DIR}/lib${PROJECT_NAME}_boot.so)

## extensions
add_library(${PROJECT_NAME}_core
    src/generic_ros_message.cpp

    src/ros_node.cpp
    src/ros_handler.cpp
    src/ros_message_conversion.cpp
    src/tf_listener.cpp    
    src/actionlib_node.cpp
)

target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

## core plugins
add_library(${PROJECT_NAME}_plugin_core
    src/apex_ros_interface.cpp
    src/bag_provider.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_plugin_core ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

## node plugins
add_library(${PROJECT_NAME}_plugin_node
    src/ros_param.cpp
    src/export_ros.cpp
    src/import_ros.cpp
    src/ros_test.cpp
    src/ros_trigger.cpp
    src/bag_writer.cpp
    src/ros_message_bool_2_event.cpp
    src/ros_message_int_2_event.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_plugin_node ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

## qt plugins
add_library(${PROJECT_NAME}_plugin_qt
    src/file_drag_io.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_plugin_qt ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)


#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS
          ${PROJECT_NAME}_boot
          ${PROJECT_NAME}_core
          ${PROJECT_NAME}_plugin_core
          ${PROJECT_NAME}_plugin_node
          ${PROJECT_NAME}_plugin_qt
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})