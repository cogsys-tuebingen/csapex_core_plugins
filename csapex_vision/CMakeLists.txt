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
find_package(catkin REQUIRED COMPONENTS cslibs_vision csapex_opencv csapex csapex_ros csapex_core_plugins cv_bridge csapex_ml)

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
    src/plugin/register_plugin.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin_core
    yaml-cpp ${QT_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${catkin_LIBRARIES})


add_library(${PROJECT_NAME}_plugin_node
    src/plugin/static_roi.cpp
    src/plugin/rotate_image.cpp
    src/plugin/grab_cut.cpp
    src/plugin/camera.cpp
    src/plugin/image_padding.cpp
    src/plugin/copymakeborder.cpp
    src/plugin/color_segmentation.cpp
    src/plugin/filter/filter_apply_mask.cpp

    src/plugin/filter/filter_static_mask.cpp

    src/plugin/mask_refinement.cpp

    src/plugin/hough_circle.cpp
    src/plugin/vectorize_pyramid.cpp

    src/plugin/image_roi.cpp
    src/plugin/grid_arranged_rois.cpp

    src/plugin/row.cpp
    src/plugin/col.cpp
    src/plugin/minmax_loc.cpp
    src/plugin/set_color.cpp
    src/plugin/reshape.cpp
    src/plugin/mean_dev_normalization.cpp

    src/plugin/levels.cpp
    src/plugin/blur.cpp
    src/plugin/gaussian_blur.cpp
    src/plugin/bilateral.cpp
    src/plugin/mean_dev.cpp

    src/plugin/load_filestorage.cpp


    src/plugin/filter/distance_transform.cpp
    src/plugin/filter/gamma_correction.cpp
    src/plugin/filter/filter_merger.cpp
    src/plugin/filter/filter_splitter.cpp
    src/plugin/filter/filter_perspective.cpp

    src/plugin/filter/filter_debayer.cpp
    src/plugin/filter/morphological.cpp
#    src/plugin/vj_detector.cpp
    src/plugin/number_generator.cpp

    src/plugin/roi/merge_rois.cpp
    src/plugin/roi/grow_roi.cpp
    src/plugin/roi/grow_rois.cpp
    src/plugin/roi/extract_roi.cpp
    src/plugin/roi/filter_rois.cpp
    src/plugin/roi/flip_rois.cpp

    src/plugin/combine/generic_image_combiner.cpp
    src/plugin/combine/set_operation.cpp
    src/plugin/combine/sum_channels.cpp
    src/plugin/combine/matrix_stitcher.cpp

    src/plugin/modify/undistort.cpp
    src/plugin/modify/flip.cpp
    src/plugin/modify/scale.cpp
    src/plugin/modify/resize.cpp
    src/plugin/modify/float_to_uchar.cpp
    src/plugin/modify/convert_type.cpp

    src/plugin/color/color_convert.cpp
    src/plugin/color/color_adjustment.cpp
    src/plugin/color/yuv422_decoder.cpp
    src/plugin/color/contrast_brightness.cpp

    src/plugin/features/corner_line_detection.cpp
    src/plugin/features/difference_maximum.cpp
    src/plugin/features/harris.cpp
    src/plugin/features/canny.cpp
    src/plugin/features/eigenvals_and_vecs.cpp
    src/plugin/features/hough_linesp.cpp
    src/plugin/features/hough_linesp.cpp
    src/plugin/features/operator.cpp
    src/plugin/features/laplacian.cpp
    src/plugin/features/sobel.cpp
    src/plugin/features/scharr.cpp
    src/plugin/features/lsd.cpp

    src/plugin/visualize/matrix_to_heatmap.cpp
    src/plugin/visualize/render_labels.cpp
    src/plugin/visualize/image_text_label.cpp
    src/plugin/visualize/render_rois.cpp

    src/plugin/filter/median_filter.cpp
    src/plugin/filter/binomial_filter.cpp
    src/plugin/filter/monofilter.cpp
    src/plugin/filter/sequence_mean.cpp
    src/plugin/filter/equalize.cpp
    src/plugin/filter/normalize.cpp
    src/plugin/filter/adaptive_threshold.cpp
    src/plugin/filter/threshold_noise_filter.cpp

    src/plugin/analyze/pyramid.cpp
    src/plugin/analyze/camera_calibration.cpp
    src/plugin/analyze/cluster_boundary_mask.cpp
    src/plugin/analyze/glcm.cpp

    src/plugin/segment/preemptiveSLIC.cpp

    src/plugin/label/label_regions.cpp
    src/plugin/label/cluster_grid.cpp
    src/plugin/label/assign_cluster_class.cpp
    src/plugin/label/assign_roi_class.cpp

    src/plugin/recognition/acf_standard_extractor.cpp
    src/plugin/recognition/acf_dynamic_extractor.cpp
    src/plugin/recognition/hog_detector.cpp
    src/plugin/recognition/hog_extractor.cpp
    src/plugin/recognition/hog.cpp
    src/plugin/recognition/hog_classifier.cpp
    src/plugin/recognition/local_patterns.cpp
    src/plugin/recognition/local_patterns_histogram.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin_node
    yaml-cpp ${QT_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${catkin_LIBRARIES})


add_library(${PROJECT_NAME}_plugin_qt
    src/plugin/filter/filter_static_mask_painter.cpp
    src/plugin/filter/filter_static_mask_adapter.cpp
    src/plugin/mask_refinement_adapter.cpp
    src/plugin/image_roi_adapter.cpp

    src/plugin/label/assign_cluster_class_adapter.cpp
    src/plugin/label/assign_roi_class_adapter.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin_qt
    yaml-cpp ${QT_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${catkin_LIBRARIES})


#add_executable(vision_evaluator src/deprecated.cpp)

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

