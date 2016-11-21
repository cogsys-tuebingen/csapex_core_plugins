cmake_minimum_required(VERSION 2.8.3)
project(csapex_point_cloud)

ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
add_definitions("-O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        csapex
        csapex_ros
        csapex_transform
        csapex_opencv
        cslibs_kdtree
)

find_package(pcl_ros REQUIRED)

find_package(Qt5 COMPONENTS Core OpenGL Gui Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)

find_package(PCL REQUIRED COMPONENTS common filters segmentation)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


catkin_package(
  INCLUDE_DIRS include ${PCL_INCLUDE_DIRS}
  LIBRARIES csapex_point_cloud
  CATKIN_DEPENDS csapex
#  DEPENDS system_lib
)


include_directories(
  include
  src/math/
  ${catkin_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

add_library(csapex_point_cloud
    src/msg/point_cloud_message.cpp
    src/msg/model_message.cpp
    src/msg/normals_message.cpp
    src/msg/indeces_message.cpp
)
target_link_libraries(csapex_point_cloud
    yaml-cpp ${catkin_LIBRARIES})


add_library(csapex_point_cloud_plugin_core
    src/register_plugin.cpp
)

target_link_libraries(csapex_point_cloud_plugin_core
    yaml-cpp ${QT_LIBRARIES} ${catkin_LIBRARIES} Qt5::OpenGL
    csapex_point_cloud ${PCL_COMMON_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES}
)

add_library(csapex_point_cloud_plugin_node
    src/project_z.cpp
    src/merge_clouds.cpp
    src/point_count.cpp
    src/cloud_renderer.cpp
    src/transform_cloud.cpp
    src/voxel_grid.cpp
    src/extract_timestamp.cpp
    src/set_timestamp.cpp
    src/row.cpp
    src/col.cpp
    src/roi_generator.cpp

    src/cloud_test.cpp

    src/labeling/cloud_labeler.cpp

    src/conversion/pointcloud_to_depthimage.cpp
    src/conversion/pointcloud_to_colorimage.cpp
    src/conversion/pointcloud_to_intensityimage.cpp
    src/conversion/pointcloud_to_pointmatrix.cpp
    src/conversion/pointmatrix_to_pointcloud.cpp
    src/conversion/image_to_point_cloud.cpp
    src/conversion/pointcloud_to_xyz.cpp
    src/conversion/indices_to_mask.cpp
    src/conversion/indices_to_rois.cpp
    src/conversion/normals_to_colorimage.cpp

    src/filter/conditional_outlier_removal.cpp
    src/filter/statistical_outlier_removal.cpp
    src/filter/radius_outlier_removal.cpp
    src/filter/threshold_outlier_revoval.cpp
    src/filter/pointcloud_validity.cpp
    src/filter/coordinate_swapper.cpp
    src/filter/indexed_pointcloud.cpp
    src/filter/index_filter.cpp
    src/filter/edge_filter.cpp
    src/filter/roi_extractor.cpp
    src/filter/passthrough.cpp
    src/filter/passthrough_by_cluster.cpp
    src/filter/passthrough_by_indices.cpp
    src/filter/crop_box.cpp
    src/filter/filter_rois_by_distance.cpp

    src/segmentation/labeled_cloud_to_indices.cpp
    src/segmentation/label_pointcloud.cpp
    src/segmentation/cluster_pointcloud.cpp
    src/segmentation/color_labeled_point_cloud.cpp
    src/segmentation/split_clustered_cloud.cpp
    src/segmentation/label_clustered_pointcloud.cpp
    src/segmentation/merge_clusters.cpp

    src/segmentation/cluster_kdtree.cpp
    src/segmentation/cluster_kdtree_filtered.cpp

    src/segmentation/regular_structures/entry.hpp
    src/segmentation/regular_structures/validator.hpp
    src/segmentation/regular_structures/indexation.hpp
    src/segmentation/regular_structures/cluster_params.hpp
    src/segmentation/regular_structures/clustering.hpp
    src/segmentation/regular_structures/filtered_clustering.hpp
    src/segmentation/regular_structures/filtered_clustering_color.hpp
    src/segmentation/regular_structures/filtered_clustering_mono.hpp
    src/segmentation/cluster_regular.cpp
    src/segmentation/cluster_regular_filtered.cpp
    src/segmentation/cluster_regular_filtered_color.cpp
    src/segmentation/cluster_regular_filtered_mono.cpp
    src/segmentation/extract_clusters.cpp

    src/analysis/ray_angles.cpp
    src/analysis/estimate_normals.cpp
    src/analysis/estimate_center.cpp

    src/math/distribution.hpp
    src/math/mean.hpp

    src/color_point_cloud.cpp
    src/pointcloud_generator.cpp
    src/sac_fit.cpp
    src/model_to_marker.cpp
    src/transform_from_models.cpp
    src/scale_intensity.cpp
    src/clusters_to_markerarray.cpp

    ${clustering_kd_tree_SRCS}
)

target_link_libraries(csapex_point_cloud_plugin_node
    yaml-cpp ${QT_LIBRARIES} ${catkin_LIBRARIES} Qt5::OpenGL
    csapex_point_cloud ${PCL_COMMON_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES}
)

add_library(csapex_point_cloud_plugin_qt
    src/point_count_adapter.cpp
    src/cloud_renderer_adapter.cpp
    src/labeling/cloud_labeler_adapter.cpp
)

target_link_libraries(csapex_point_cloud_plugin_qt
    yaml-cpp ${QT_LIBRARIES} ${catkin_LIBRARIES} Qt5::OpenGL
    csapex_point_cloud ${PCL_COMMON_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES}
)


#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS csapex_point_cloud
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_point_cloud_plugin_core
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS csapex_point_cloud_plugin_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS csapex_point_cloud_plugin_qt
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

