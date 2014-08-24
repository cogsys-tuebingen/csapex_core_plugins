cmake_minimum_required(VERSION 2.8.3)
project(csapex_ml)

find_package(catkin REQUIRED COMPONENTS
  csapex
  csapex_core_plugins
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES csapex_ml csapex_ml_plugin
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(csapex_ml
    src/features_message.cpp
)
target_link_libraries(csapex_ml
    yaml-cpp ${Boost_LIBRARIES} ${catkin_LIBRARIES})


add_library(csapex_ml_plugin
    src/decision_tree.cpp
    src/decision_tree_trainer.cpp
)

target_link_libraries(csapex_ml_plugin
    yaml-cpp csapex_ml ${catkin_LIBRARIES})

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
