cmake_minimum_required(VERSION 2.8.3)
project(vision_plugins_conversion)

find_package(catkin REQUIRED COMPONENTS
  csapex
  csapex_ml
  csapex_vision
  csapex_vision_features
  vision_plugins_histograms
)

set(CMAKE_BUILD_TYPE Release)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vision_plugins_conversion
#  CATKIN_DEPENDS csapex csapex_ml csapex_vision csapex_vision_features
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(vision_plugins_conversion
    src/feature_to_histogram.cpp
    src/to_feature.cpp
)

target_link_libraries(vision_plugins_conversion
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS vision_plugins_conversion
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

