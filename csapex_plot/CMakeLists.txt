cmake_minimum_required(VERSION 2.8.3)
project(csapex_plot)


include(ExternalProject)

ExternalProject_add(QWT
  SVN_REPOSITORY svn://svn.code.sf.net/p/qwt/code/trunk/qwt
  SVN_REVISION -r "2365"
  PATCH_COMMAND ""
  UPDATE_COMMAND ""
  SOURCE_DIR QWT
  BINARY_DIR QWT-build
  "${cmakeversion_external_update}"
  PATCH_COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt.qwt
  <SOURCE_DIR>/CMakeLists.txt
  CMAKE_ARGS ${COMMON_EXTERNAL_PROJECT_ARGS}
  -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_CURRENT_BINARY_DIR}
  INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}
)

set(QWT_LIBRARY ${CMAKE_CURRENT_BINARY_DIR}/lib/libqwt-qt5.so)

set(QWT_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include)

list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_BINARY_DIR} )



find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS
  csapex
  csapex_vision
)

catkin_package(
    INCLUDE_DIRS ${QWT_INCLUDE_DIR}
    LIBRARIES ${QWT_LIBRARY}
)

include_directories(
  ${catkin_INCLUDE_DIRS} ${QWT_INCLUDE_DIR}
  ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
)

add_library(csapex_plot_node
    src/time_plot.cpp
)
target_link_libraries(csapex_plot_node
    ${catkin_LIBRARIES} ${QWT_LIBRARY} ${QT_LIBRARIES}
)
add_dependencies(csapex_plot_node QWT)


add_library(csapex_plot_plugin_qt
    src/time_plot_adapter.cpp
)
target_link_libraries(csapex_plot_plugin_qt
    yaml-cpp ${catkin_LIBRARIES})
add_dependencies(csapex_plot_plugin_qt QWT)
