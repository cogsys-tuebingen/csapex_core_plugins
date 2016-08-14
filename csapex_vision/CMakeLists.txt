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
find_package(catkin REQUIRED COMPONENTS cslibs_vision csapex csapex_ros csapex_core_plugins cv_bridge csapex_ml)

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
   INCLUDE_DIRS include src
   LIBRARIES csapex_vision
   CATKIN_DEPENDS csapex cslibs_vision csapex_core_plugins
   DEPENDS OpenCV
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

qt5_add_resources(QT_RESOURCES res/csapex_vision_resources.qrc)


add_subdirectory(src/cvBlob)


add_library(csapex_vision
    src/core/filter.cpp
    src/core/encoding.cpp
    src/core/cv_mat_message.cpp
    src/core/roi.cpp
    src/core/roi_message.cpp
    src/core/cv_pyramid_message.cpp
    src/core/yaml_io.cpp

    ${QT_RESOURCES}
)
target_link_libraries(csapex_vision
    yaml-cpp ${QT_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${catkin_LIBRARIES})



add_library(csapex_vision_plugin_core
    src/plugin/register_plugin.cpp
    src/core/image_renderer.cpp
    src/core/image_provider.cpp
    src/plugin/image_provider_dir.cpp
    src/plugin/image_provider_img.cpp
    src/plugin/image_provider_mov.cpp
    src/plugin/image_provider_set.cpp
)
target_link_libraries(csapex_vision_plugin_core csapex_vision
    yaml-cpp ${QT_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${catkin_LIBRARIES})


add_library(csapex_vision_plugin_node
    src/plugin/static_roi.cpp
    src/plugin/rotate_image.cpp
    src/plugin/grab_cut.cpp
    src/plugin/camera.cpp
    src/plugin/image_padding.cpp
    src/plugin/copymakeborder.cpp
    src/plugin/color_segmentation.cpp
    src/plugin/filter_apply_mask.cpp

    src/plugin/filter_static_mask.cpp

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

    src/plugin/example_image_generator.cpp




    src/filter/distance_transform.cpp
    src/filter/gamma_correction.cpp
    src/blob_detector.cpp
    src/filter_merger.cpp
    src/filter_splitter.cpp
    src/filter_perspective.cpp

    src/filter_debayer.cpp
    src/morphological.cpp
#    src/vj_detector.cpp
    src/lk_tracking.cpp
    src/number_generator.cpp

    src/roi/merge_rois.cpp
    src/roi/grow_roi.cpp
    src/roi/grow_rois.cpp
    src/roi/extract_roi.cpp
    src/roi/filter_rois.cpp
    src/roi/flip_rois.cpp

    src/combine/generic_image_combiner.cpp
    src/combine/set_operation.cpp
    src/combine/sum_channels.cpp
    src/combine/matrix_stitcher.cpp

    src/modify/undistort.cpp
    src/modify/flip.cpp
    src/modify/scale.cpp
    src/modify/resize.cpp
    src/modify/float_to_uchar.cpp
    src/modify/convert_type.cpp

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
    src/label/assign_roi_class.cpp

    src/recognition/hog_detector.cpp
    src/recognition/hog_extractor.cpp
    src/recognition/hog.cpp
    src/recognition/hog_classifier.cpp
    src/recognition/local_patterns.cpp
    src/recognition/local_patterns_histogram.cpp
)
target_link_libraries(csapex_vision_plugin_node csapex_vision
    cvblob
    yaml-cpp ${QT_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${catkin_LIBRARIES})


add_library(csapex_vision_plugin_qt
    src/plugin/filter_static_mask_painter.cpp
    src/plugin/filter_static_mask_adapter.cpp
    src/plugin/mask_refinement_adapter.cpp
    src/plugin/image_roi_adapter.cpp

    src/label/assign_cluster_class_adapter.cpp
    src/label/assign_roi_class_adapter.cpp
)
target_link_libraries(csapex_vision_plugin_qt csapex_vision
    yaml-cpp ${QT_LIBRARIES} ${OpenCV_LIBS} ${OPENGL_LIBRARIES} ${catkin_LIBRARIES})


#add_executable(vision_evaluator src/deprecated.cpp)

#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS csapex_vision
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_vision_plugin_core
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS csapex_vision_plugin_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS csapex_vision_plugin_qt
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

