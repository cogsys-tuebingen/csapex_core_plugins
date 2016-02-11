cmake_minimum_required(VERSION 2.8.3)
project(csapex_evaluation)

find_package(catkin REQUIRED COMPONENTS
  csapex
  csapex_ml
  csapex_plot
)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES csapex_evaluation
  CATKIN_DEPENDS csapex_ml
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
)
add_library(csapex_evaluation
    src/confusion_matrix.cpp
    src/confusion_matrix_message.cpp
    src/confidence_matrix.cpp
    src/confidence_matrix_message.cpp
)
target_link_libraries(csapex_evaluation
    yaml-cpp ${Boost_LIBRARIES} ${catkin_LIBRARIES})


add_library(csapex_evaluation_plugin_node
    src/roc/roc_curve.cpp

    src/confusion_matrix_display.cpp
    src/confidence_matrix_display.cpp
    src/evaluate_binary_classifier.cpp
    src/empty_labeled_feature_message.cpp
    src/confidence_evaluator.cpp
    src/ml_evaluator.cpp
    src/confidence_class_toggle.cpp
)

target_link_libraries(csapex_evaluation_plugin_node csapex_evaluation
    yaml-cpp ${catkin_LIBRARIES})


add_library(csapex_evaluation_plugin_qt
    src/roc/roc_curve_adapter.cpp

    src/confusion_matrix_display_adapter.cpp
    src/confidence_matrix_display_adapter.cpp
    src/evaluate_binary_classifier_adapter.cpp
)

target_link_libraries(csapex_evaluation_plugin_qt csapex_evaluation
    yaml-cpp ${catkin_LIBRARIES})

#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS csapex_evaluation_plugin_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_evaluation_plugin_qt
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_evaluation
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
