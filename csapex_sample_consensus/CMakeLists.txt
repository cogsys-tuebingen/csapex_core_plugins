cmake_minimum_required(VERSION 2.8.3)
project(csapex_sample_consensus)

add_definitions(-DQT_NO_KEYWORDS)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
#add_definitions("-O3")

find_package(catkin REQUIRED COMPONENTS
    csapex
    csapex_core_plugins
    csapex_point_cloud
    )

find_package(PCL REQUIRED COMPONENTS common filters segmentation io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    #  LIBRARIES csapex_sample_consensus
    #  CATKIN_DEPENDS csapex csapex_core_plugins csapex_point_cloud pcl
    #  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_plugin_node
    include/csapex_sample_consensus/algorithms/sac.hpp
    include/csapex_sample_consensus/algorithms/antsac.hpp
    include/csapex_sample_consensus/algorithms/ransac.hpp

    include/csapex_sample_consensus/models/sac_model.hpp
    include/csapex_sample_consensus/models/sac_model_from_normals.hpp
    include/csapex_sample_consensus/models/sac_model_plane.hpp
    include/csapex_sample_consensus/models/sac_model_normal_plane.hpp
    include/csapex_sample_consensus/models/sac_model_parallel_normal_plane.hpp

    src/sample_consensus.hpp
    src/antsac.cpp
    src/ransac.cpp
)

target_link_libraries(${PROJECT_NAME}_plugin_node
    ${catkin_LIBRARIES}
    ${PCL_COMMON_LIBRARIES}
    ${PCL_FILTERS_LIBRARIES}
    ${PCL_SEGMENTATION_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    )

#
# INSTALL
#

install(FILES plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_plugin_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
