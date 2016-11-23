cmake_minimum_required(VERSION 2.8.3)
project(csapex_windows)

find_package(csapex REQUIRED)

csapex_package()

## booting
include_directories(include 
    ${csapex_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}_boot SHARED
    src/windows_boot.cpp
)
target_link_libraries(${PROJECT_NAME}_boot
    ${catkin_LIBRARIES}
    ${csapex_LIBRARIES}
)

#
# INSTALL
#

install(TARGETS ${PROJECT_NAME}_boot
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
