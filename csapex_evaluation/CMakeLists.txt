cmake_minimum_required(VERSION 2.8.3)
project(csapex_evaluation)

find_package(catkin REQUIRED COMPONENTS
  csapex_ml
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES csapex_evaluation csapex_evaluation_plugin
  CATKIN_DEPENDS csapex_ml
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(csapex_evaluation
    src/confusion_matrix.cpp
    src/confusion_matrix_message.cpp
)
target_link_libraries(csapex_evaluation
    yaml-cpp ${Boost_LIBRARIES} ${catkin_LIBRARIES})


add_library(csapex_evaluation_plugin
    src/confusion_matrix_display.cpp
    src/confusion_matrix_display_adapter.cpp
    src/evaluate_binary_classifier.cpp
    src/evaluate_binary_classifier_adapter.cpp
    src/ml_evaluator.cpp
)

target_link_libraries(csapex_evaluation_plugin csapex_evaluation
    yaml-cpp csapex_ml ${catkin_LIBRARIES})

#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS csapex_evaluation_plugin
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_evaluation
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
