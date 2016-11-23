cmake_minimum_required(VERSION 2.8.11)
project(csapex_optimization)

find_package(catkin REQUIRED COMPONENTS
  csapex
)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS csapex
)

include_directories(
  include
  ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/parameter_dialog.cpp
    include/${PROJECT_NAME}/parameter_dialog.h
)
target_link_libraries(${PROJECT_NAME}
    yaml-cpp ${Boost_LIBRARIES} ${catkin_LIBRARIES}
)


add_library(${PROJECT_NAME}_plugin_node
    src/optimization_dummy.cpp
    src/bf_optimizer.cpp
)

target_link_libraries(${PROJECT_NAME}_plugin_node
    yaml-cpp ${PROJECT_NAME} ${catkin_LIBRARIES})


add_library(${PROJECT_NAME}_plugin_qt
    src/parameter_dialog.cpp
    src/bf_optimizer_adapter.cpp
)

target_link_libraries(${PROJECT_NAME}_plugin_qt
    yaml-cpp ${PROJECT_NAME} ${catkin_LIBRARIES})

#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugin_node ${PROJECT_NAME}_plugin_qt
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
