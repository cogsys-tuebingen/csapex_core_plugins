cmake_minimum_required(VERSION 2.8.3)
project(vision_plugins)

find_package(catkin REQUIRED roscpp csapex_vision csapex_vision_features utils_cv utils_viola_jones utils_vision)

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
  INCLUDE_DIRS include
  LIBRARIES cvblob
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src/cvBlob)

add_library(vision_plugins
    src/adaptive_threshold.cpp
    src/blob_detector.cpp
    src/generic_image_combiner.cpp
    src/combiner_gridcompare.cpp
    src/combiner_gridcompare_hist.cpp
    src/combiner_gridheatmap_hist.cpp
    src/combiner_gridcompare_value.cpp
    src/combiner_gridheatmap_value.cpp
    src/combiner_set_operation.cpp
    src/filter_merger.cpp
    src/filter_splitter.cpp
    src/filter_perspective.cpp
    src/filter_histogram.cpp
    src/filter_coloradjustment.cpp
    src/filter_colorconvert.cpp
    src/filter_debayer.cpp
    src/filter_local_patterns.cpp
    src/morphological.cpp
    src/vj_detector.cpp
    src/lk_tracking.cpp
    src/visualize/render_rois.cpp
    src/merge_rois.cpp
    src/extract_roi.cpp
    src/simple_sink.cpp
    src/number_generator.cpp
   ${MOC}

    ###  NEW API ###

    src/modify/undistort.cpp
    src/modify/flip.cpp
    src/modify/scale.cpp
    src/modify/resize.cpp

    src/corners_lines/detection.cpp
    src/corners_lines/harris.cpp
    src/corners_lines/canny.cpp
    src/corners_lines/eigenvals_and_vecs.cpp
    src/corners_lines/hough_linesp.cpp
    src/corners_lines/hough_linesp.cpp
    src/corners_lines/operator.cpp
    src/corners_lines/laplacian.cpp
    src/corners_lines/sobel.cpp
    src/corners_lines/scharr.cpp

    src/visualize/matrix_to_heatmap.cpp

    src/filter/median_filter.cpp
    src/filter/monofilter.cpp
    src/filter/sequence_mean.cpp
    src/filter/equalize.cpp

    src/analysis/pyramid.cpp
    src/analysis/camera_calibration.cpp

    src/convert_type.cpp
    src/float_to_uchar.cpp

)

target_link_libraries(vision_plugins yaml-cpp ${OpenCV_LIBS} ${Boost_LIBRARIES}
  ${catkin_LIBRARIES} cvblob
)
