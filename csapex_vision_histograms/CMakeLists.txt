cmake_minimum_required(VERSION 3.8.2)
project(csapex_vision_histograms CXX)

find_package(catkin COMPONENTS
                    roscpp
                    csapex
                    csapex_opencv
                    csapex_vision_features
                    csapex_ml
                    cslibs_vision
                    REQUIRED)

ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

#set(CMAKE_CXX_FLAGS "-O3")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS  include
  LIBRARIES     ${PROJECT_NAME}_msgs
  CATKIN_DEPENDS csapex_opencv
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_msgs
    src/msg/histogram_msg.cpp
    src/msg/histogram_container.cpp
    src/msg/histogram_maxima_msg.cpp
    src/msg/histogram_maxima_container.cpp
)

target_link_libraries(${PROJECT_NAME}_msgs
    ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}
    src/visualize/render_histogram.cpp
    src/analyze/histogram.cpp
    src/analyze/cluster_histograms.cpp
    src/analyze/histogram_maxima.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_msgs
  yaml-cpp
  ${catkin_LIBRARIES} 
)

#############
## Install ##
#############

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_msgs ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

