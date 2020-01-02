cmake_minimum_required(VERSION 3.8.2)
project(csapex_evaluation CXX)

find_package(catkin REQUIRED COMPONENTS
  csapex
  csapex_ml
  csapex_plot
)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS csapex_ml csapex_plot
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
    src/confusion_matrix.cpp
    src/confusion_matrix_message.cpp
    src/confidence_matrix.cpp
    src/confidence_matrix_message.cpp
)
target_link_libraries(${PROJECT_NAME}
    yaml-cpp ${Boost_LIBRARIES} ${catkin_LIBRARIES})


add_library(${PROJECT_NAME}_plugin_node
    src/roc/roc_curve.cpp

    src/confusion_matrix_display.cpp
    src/confidence_matrix_display.cpp
    src/evaluate_binary_classifier.cpp
    src/empty_labeled_feature_message.cpp
    src/confidence_evaluator.cpp
    src/evaluate_feature_classification.cpp
    src/confidence_class_toggle.cpp
)

target_link_libraries(${PROJECT_NAME}_plugin_node ${PROJECT_NAME}
    yaml-cpp ${catkin_LIBRARIES})


add_library(${PROJECT_NAME}_plugin_qt
    src/roc/roc_curve_adapter.cpp

    src/confusion_matrix_display_adapter.cpp
    src/confidence_matrix_display_adapter.cpp
    src/evaluate_binary_classifier_adapter.cpp
)

target_link_libraries(${PROJECT_NAME}_plugin_qt ${PROJECT_NAME}
    yaml-cpp ${catkin_LIBRARIES})

#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_plugin_node ${PROJECT_NAME}_plugin_qt ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
