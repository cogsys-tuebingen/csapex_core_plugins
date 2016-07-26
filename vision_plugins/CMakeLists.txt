cmake_minimum_required(VERSION 2.8.11)
project(vision_plugins)

find_package(catkin COMPONENTS roscpp csapex csapex_vision csapex_vision_features csapex_ml utils_vision REQUIRED)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)

ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

find_package(OpenMP QUIET)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")


#set(CMAKE_BUILD_TYPE Debug)
add_definitions("-O3")

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
)

###########
## Build ##
###########

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
)

add_subdirectory(src/cvBlob)

add_library(vision_plugins_node

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

    src/extract_roi.cpp
    src/number_generator.cpp

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
    src/modify/flip_rois.cpp
    src/modify/merge_rois.cpp
    src/modify/grow_roi.cpp
    src/modify/grow_rois.cpp

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
    src/filter/filter_rois.cpp

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

target_link_libraries(vision_plugins_node
  yaml-cpp 
  ${OpenCV_LIBS} 
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES} 
  cvblob
)

add_library(vision_plugins_qt
    src/label/assign_cluster_class_adapter.cpp
    src/label/assign_roi_class_adapter.cpp
)

target_link_libraries(vision_plugins_qt
  yaml-cpp
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  cvblob
)


#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS vision_plugins_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS vision_plugins_qt
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

