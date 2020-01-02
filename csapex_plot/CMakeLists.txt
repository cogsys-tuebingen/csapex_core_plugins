cmake_minimum_required(VERSION 2.8.3)
project(csapex_plot)

## Add our own cmake subdirectory
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS
  csapex
  csapex_opencv
)

find_package(Qwt REQUIRED)

set(QWT_LIBRARIES ${QWT_LIBRARY})
set(QWT_INCLUDE_DIRS ${QWT_INCLUDE_DIR})

catkin_package(
    DEPENDS QWT
)

include_directories(
  ${catkin_INCLUDE_DIRS} ${QWT_INCLUDE_DIR}
  ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}_node
    src/plot.cpp

    src/time_plot.cpp
    src/vector_plot.cpp
    src/scatter_plot.cpp
)
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES} ${QWT_LIBRARY} ${QT_LIBRARIES}
)


add_library(${PROJECT_NAME}_plugin_qt
    src/time_plot_adapter.cpp
    src/vector_plot_adapter.cpp
    src/scatter_plot_adapter.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin_qt
    yaml-cpp ${catkin_LIBRARIES})



install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_node
                ${PROJECT_NAME}_plugin_qt
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY ${QWT_INCLUDE_DIR}/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})