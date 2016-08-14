cmake_minimum_required(VERSION 2.8.3)
project(csapex_opencv)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS csapex)

# note to ubuntu 14 users: if opencv is installed at /usr/local and is not found:    sudo ldconfig /usr/local/lib
set(USED_OpenCV_COMPONENTS core imgproc ml calib3d)
find_package(OpenCV COMPONENTS nonfree ${USED_OpenCV_COMPONENTS} QUIET CONFIG)

list(FIND OpenCV_LIB_COMPONENTS opencv_nonfree NONFREE_INDEX)
set(NONFREE_FOUND NOT ${NONFREE_INDEX} EQUAL -1)

if(${OpenCV_FOUND} AND ${NONFREE_FOUND})
    add_definitions("-DCV_NON_FREE=1")
else()
    add_definitions("-DCV_NON_FREE=0")
    find_package(OpenCV REQUIRED COMPONENTS ${USED_OpenCV_COMPONENTS} CONFIG)
endif()

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES csapex_opencv
  DEPENDS OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
)

add_subdirectory(src/cvBlob)

qt5_add_resources(QT_RESOURCES res/csapex_vision_resources.qrc)

add_library(${PROJECT_NAME}
    ${QT_RESOURCES}

    src/filter.cpp
    src/encoding.cpp
    src/cv_mat_message.cpp
    src/roi.cpp
    src/roi_message.cpp
    src/cv_pyramid_message.cpp
    src/yaml_io.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS})


add_library(${PROJECT_NAME}_core
    src/image_renderer.cpp
    src/image_provider.cpp
    src/image_provider_dir.cpp
    src/image_provider_img.cpp
    src/image_provider_mov.cpp
    src/image_provider_set.cpp
)
target_link_libraries(${PROJECT_NAME}_core
    ${PROJECT_NAME}
   yaml-cpp  ${OpenCV_LIBS} ${catkin_LIBRARIES} ${QT_LIBRARIES})


add_library(${PROJECT_NAME}_node
    src/example_image_generator.cpp
    src/blob_detector.cpp
)
target_link_libraries(${PROJECT_NAME}_node
    cvblob
    ${PROJECT_NAME}
   yaml-cpp  ${OpenCV_LIBS} ${catkin_LIBRARIES})

#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


install(TARGETS ${PROJECT_NAME}_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS ${PROJECT_NAME}_core
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

