cmake_minimum_required(VERSION 2.8.3)
project(vision_plugins_histograms)

find_package(catkin COMPONENTS roscpp csapex csapex_vision csapex_vision_features utils_cv utils_viola_jones utils_vision REQUIRED)

ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

set(CMAKE_BUILD_TYPE Debug)

#set(CMAKE_CXX_FLAGS "-O3")

csapex_moc_all(MOC src/*.h)

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
  LIBRARIES     vision_plugins_histograms_msgs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

add_library(vision_plugins_histograms_msgs
    src/msg/histogram_msg.cpp
    src/msg/histogram_container.cpp
    src/msg/histogram_maxima_msg.cpp
    src/msg/histogram_maxima_container.cpp
)

target_link_libraries(vision_plugins_histograms_msgs
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_library(vision_plugins_histograms
    src/visualize/render_histogram.cpp
    src/analyze/histogram.cpp
    src/analyze/histogram_maxima.cpp
)

target_link_libraries(vision_plugins_histograms 
  yaml-cpp 
  vision_plugins_histograms_msgs
  ${OpenCV_LIBS} ${Boost_LIBRARIES}
  ${catkin_LIBRARIES} 
)



#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS vision_plugins_histograms
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

