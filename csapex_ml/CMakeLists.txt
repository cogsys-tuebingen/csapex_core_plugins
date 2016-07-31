cmake_minimum_required(VERSION 2.8.3)
project(csapex_ml)

find_package(catkin REQUIRED COMPONENTS
  csapex
  csapex_core_plugins
  csapex_vision
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES csapex_ml
  CATKIN_DEPENDS csapex csapex_core_plugins
)

find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUED_DIRS}
)
add_library(csapex_ml
    src/features_message.cpp
)
target_link_libraries(csapex_ml
    yaml-cpp ${Boost_LIBRARIES} ${catkin_LIBRARIES})


add_library(csapex_ml_plugin
    src/decision_tree.cpp
    src/decision_tree_trainer.cpp

    src/random_trees.cpp
    src/random_trees_trainer.cpp

    src/svm_trainer.cpp
    src/svm.cpp
    src/svm_array_trainer.cpp
    src/svm_array.cpp

    src/mlp.cpp
    src/mlp/mlp.cpp

    src/mlp_cv.cpp
    src/mlp_cv_trainer.cpp

    src/arff_format_export.cpp

    src/assign_class.cpp
)

target_link_libraries(csapex_ml_plugin
    yaml-cpp csapex_ml ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS csapex_ml_plugin
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_ml
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
