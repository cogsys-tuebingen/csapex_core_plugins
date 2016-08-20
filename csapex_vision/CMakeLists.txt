cmake_minimum_required(VERSION 2.8.3)
project(csapex_vision)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cslibs_vision
                                        csapex_opencv
                                        csapex
                                        csapex_ros
                                        csapex_core_plugins
                                        cv_bridge
                                        csapex_ml
                                        csapex_evaluation)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

find_package(OpenMP QUIET)

list(APPEND CMAKE_C_FLAGS ${OpenMP_C_FLAGS})
list(APPEND CMAKE_CXX_FLAGS ${OpenMP_CXX_FLAGS})
list(APPEND CMAKE_EXE_LINKER_FLAGS ${OpenMP_EXE_LINKER_FLAGS})

set(CMAKE_AUTOMOC ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   #LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS csapex csapex_opencv cslibs_vision csapex_core_plugins
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}_plugin_core
    src/register_plugin.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin_core
    yaml-cpp ${QT_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${catkin_LIBRARIES})


add_library(${PROJECT_NAME}_plugin_node
    src/rotate_image.cpp
    src/grab_cut.cpp
    src/camera.cpp
    src/color_segmentation.cpp
    src/filter/filter_apply_mask.cpp

    src/filter/filter_static_mask.cpp

    src/mask_refinement.cpp

    src/hough_circle.cpp
    src/vectorize_pyramid.cpp

    src/grid_arranged_rois.cpp

    src/matrix_operations/row.cpp
    src/matrix_operations/col.cpp
    src/matrix_operations/minmax_loc.cpp
    src/matrix_operations/scale.cpp
    src/matrix_operations/resize.cpp
    src/matrix_operations/flip.cpp
    src/matrix_operations/convert_type.cpp
    src/matrix_operations/reshape.cpp
    src/matrix_operations/image_padding.cpp
    src/matrix_operations/copymakeborder.cpp
    src/matrix_operations/merge.cpp
    src/matrix_operations/split.cpp

    src/set_color.cpp
    src/mean_dev_normalization.cpp

    src/levels.cpp
    src/blur.cpp
    src/gaussian_blur.cpp
    src/bilateral.cpp
    src/mean_dev.cpp

    src/load_filestorage.cpp


    src/filter/distance_transform.cpp
    src/filter/gamma_correction.cpp
    src/filter/filter_perspective.cpp

    src/filter/filter_debayer.cpp
    src/filter/morphological.cpp
#    src/vj_detector.cpp
    src/number_generator.cpp

    src/roi/merge_rois.cpp
    src/roi/grow_roi.cpp
    src/roi/grow_rois.cpp
    src/roi/extract_roi.cpp
    src/roi/filter_rois.cpp
    src/roi/flip_rois.cpp
    src/roi/label_rois.cpp
    src/roi/static_roi.cpp
    src/roi/image_roi.cpp
    src/roi/assign_roi_classification.cpp
    src/roi/set_roi_color_by_classification.cpp
    src/roi/evaluate_roi_classification.cpp

    src/combine/generic_image_combiner.cpp
    src/combine/set_operation.cpp
    src/combine/sum_channels.cpp
    src/combine/matrix_stitcher.cpp

    src/modify/undistort.cpp

    src/modify/float_to_uchar.cpp

    src/color/color_convert.cpp
    src/color/color_adjustment.cpp
    src/color/yuv422_decoder.cpp
    src/color/contrast_brightness.cpp

    src/features/corner_line_detection.cpp
    src/features/difference_maximum.cpp
    src/features/harris.cpp
    src/features/canny.cpp
    src/features/eigenvals_and_vecs.cpp
    src/features/hough_linesp.cpp
    src/features/hough_linesp.cpp
    src/features/operator.cpp
    src/features/laplacian.cpp
    src/features/sobel.cpp
    src/features/scharr.cpp
    src/features/lsd.cpp

    src/visualize/matrix_to_heatmap.cpp
    src/visualize/render_labels.cpp
    src/visualize/image_text_label.cpp
    src/visualize/render_rois.cpp

    src/filter/median_filter.cpp
    src/filter/binomial_filter.cpp
    src/filter/monofilter.cpp
    src/filter/sequence_mean.cpp
    src/filter/equalize.cpp
    src/filter/normalize.cpp
    src/filter/adaptive_threshold.cpp
    src/filter/threshold_noise_filter.cpp

    src/analyze/pyramid.cpp
    src/analyze/camera_calibration.cpp
    src/analyze/cluster_boundary_mask.cpp
    src/analyze/glcm.cpp

    src/segment/preemptiveSLIC.cpp

    src/label/label_regions.cpp
    src/label/cluster_grid.cpp
    src/label/assign_cluster_class.cpp

    src/recognition/acf_standard_extractor.cpp
    src/recognition/acf_dynamic_extractor.cpp
    src/recognition/hog_detector.cpp
    src/recognition/hog_extractor.cpp
    src/recognition/hog.cpp
    src/recognition/hog_classifier.cpp
    src/recognition/local_patterns.cpp
    src/recognition/local_patterns_histogram.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin_node
    yaml-cpp ${QT_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${catkin_LIBRARIES})


add_library(${PROJECT_NAME}_plugin_qt
    src/filter/filter_static_mask_painter.cpp
    src/filter/filter_static_mask_adapter.cpp
    src/mask_refinement_adapter.cpp

    src/label/assign_cluster_class_adapter.cpp

    src/roi/label_rois_adapter.cpp
    src/roi/image_roi_adapter.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin_qt
    yaml-cpp ${QT_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${catkin_LIBRARIES})

#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


install(TARGETS ${PROJECT_NAME}_plugin_core
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS ${PROJECT_NAME}_plugin_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS ${PROJECT_NAME}_plugin_qt
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

