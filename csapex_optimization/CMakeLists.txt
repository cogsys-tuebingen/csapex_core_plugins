cmake_minimum_required(VERSION 3.8.2)
project(csapex_optimization CXX)

find_package(catkin REQUIRED COMPONENTS
  csapex_core_plugins
  csapex
)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

set(CMAKE_AUTOMOC OFF)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_qt
  CATKIN_DEPENDS csapex
)

include_directories(
  include
  ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/vector_optimizer.cpp
    src/optimizer.cpp
    src/optimization_params.cpp

    include/${PROJECT_NAME}/vector_optimizer.h
    include/${PROJECT_NAME}/optimizer.h
    include/${PROJECT_NAME}/optimization_params.h
)
target_link_libraries(${PROJECT_NAME}
    yaml-cpp ${Boost_LIBRARIES} ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_qt
#    src/parameter_dialog.cpp
    src/optimizer_adapter.cpp

#    include/${PROJECT_NAME}/parameter_dialog.h
    include/${PROJECT_NAME}/optimizer_adapter.h
)
set_target_properties(${PROJECT_NAME}_qt PROPERTIES AUTOMOC TRUE)
target_link_libraries(${PROJECT_NAME}_qt
    yaml-cpp ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${QT_LIBRARIES}
)


add_library(${PROJECT_NAME}_plugin_node
    src/parameter_vector_add_noise.cpp
    src/get_best_optimization_result.cpp
    src/optimization_dummy_vector.cpp
    src/optimization_dummy.cpp
    src/bf_optimizer.cpp
)

target_link_libraries(${PROJECT_NAME}_plugin_node
    yaml-cpp ${PROJECT_NAME} ${catkin_LIBRARIES})


add_library(${PROJECT_NAME}_plugin_qt
    src/bf_optimizer_adapter.cpp
)
set_target_properties(${PROJECT_NAME}_plugin_qt PROPERTIES AUTOMOC TRUE)

target_link_libraries(${PROJECT_NAME}_plugin_qt
    yaml-cpp ${PROJECT_NAME}_qt ${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES}
)

#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugin_node ${PROJECT_NAME}_qt ${PROJECT_NAME}_plugin_qt
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
