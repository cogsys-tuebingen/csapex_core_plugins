cmake_minimum_required(VERSION 2.8.3)
project(csapex_tutorial)

find_package(catkin REQUIRED COMPONENTS
  csapex_opencv
  csapex_ros
  message_generation
)

find_package(csapex REQUIRED)

add_message_files(
  FILES
    TutorialMessage.msg
)

generate_messages(
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES csapex_tutorial
   CATKIN_DEPENDS message_runtime
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



add_library(csapex_tutorial_node
  src/user_switch.cpp
)
target_link_libraries(csapex_tutorial_node
  ${catkin_LIBRARIES}
)

install(TARGETS csapex_tutorial_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})



add_library(csapex_tutorial_core
  src/tutorial_core_plugin.cpp
)
target_link_libraries(csapex_tutorial_core
  ${catkin_LIBRARIES}
)

install(TARGETS csapex_tutorial_core
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})




add_library(csapex_tutorial
  src/tutorial_message.cpp
)
target_link_libraries(csapex_tutorial
  ${csapex_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS csapex_tutorial
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
