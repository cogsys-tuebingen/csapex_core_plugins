cmake_minimum_required(VERSION 2.8.3)
project(csapex_windows)

find_package(csapex REQUIRED)

csapex_package()

## booting
include_directories(include 
    ${csapex_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)
add_library(csapex_windows_boot SHARED
    src/windows_boot.cpp
)
target_link_libraries(csapex_windows_boot
    ${catkin_LIBRARIES}
    ${csapex_LIBRARIES}
)

#
# INSTALL
#

install(TARGETS csapex_windows_boot
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})