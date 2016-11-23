cmake_minimum_required(VERSION 2.8.3)
project(csapex_plot)

include(ExternalProject)


find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS
  csapex
  csapex_opencv
)


file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

set(QWT_LIBRARY ${CATKIN_DEVEL_PREFIX}/lib/libqwt-qt5.so)
set(QWT_INCLUDE_DIR ${CATKIN_DEVEL_PREFIX}/include)

catkin_package(
    INCLUDE_DIRS ${QWT_INCLUDE_DIR}
    LIBRARIES ${QWT_LIBRARY}
)

ExternalProject_add(QWT
  SVN_REPOSITORY svn://svn.code.sf.net/p/qwt/code/trunk/qwt
  SVN_REVISION -r "2365"
  PATCH_COMMAND ""
  UPDATE_COMMAND ""
  BUILD_COMMAND make -j3
  SOURCE_DIR QWT
  BINARY_DIR lib
  INSTALL_COMMAND make install
  "${cmakeversion_external_update}"
  PATCH_COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt.qwt
  <SOURCE_DIR>/CMakeLists.txt
  CMAKE_ARGS ${COMMON_EXTERNAL_PROJECT_ARGS}
  -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX}
  INSTALL_DIR ${CATKIN_DEVEL_PREFIX}
)


include_directories(
  ${catkin_INCLUDE_DIRS} ${QWT_INCLUDE_DIR}
  ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}_node
    src/time_plot.cpp
    src/vector_plot.cpp
)
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES} ${QWT_LIBRARY} ${QT_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_node QWT)


add_library(${PROJECT_NAME}_plugin_qt
    src/time_plot_adapter.cpp
    src/vector_plot_adapter.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin_qt
    yaml-cpp ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_plugin_qt QWT)



install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_plugin_qt
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
