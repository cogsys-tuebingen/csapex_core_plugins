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

add_library(${PROJECT_NAME}_plugin
    src/make_screenshot.cpp
    src/make_video.cpp
    src/image_collage.cpp
)

target_link_libraries(${PROJECT_NAME}_plugin
    yaml-cpp ${catkin_LIBRARIES})

#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_plugin
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

