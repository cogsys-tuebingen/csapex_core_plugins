cmake_minimum_required(VERSION 2.8.3)
project(csapex_video)

find_package(catkin REQUIRED COMPONENTS
  csapex csapex_opencv
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(csapex_video_plugin
    src/make_screenshot.cpp
    src/image_collage.cpp
)

target_link_libraries(csapex_video_plugin
    yaml-cpp ${catkin_LIBRARIES})

#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS csapex_video_plugin
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

