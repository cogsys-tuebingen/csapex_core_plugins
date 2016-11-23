cmake_minimum_required(VERSION 2.8.3)
project(csapex_rqt)

find_package(catkin REQUIRED COMPONENTS class_loader roscpp rqt_gui rqt_gui_cpp csapex)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES csapex_rqt
#  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(csapex_rqt
    src/rqt_evaluation.cpp
)
target_link_libraries(csapex_rqt
    ${catkin_LIBRARIES}
    csapex_param csapex_util csapex_qt csapex_command csapex_profiling csapex
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
    Qt5::Core Qt5::Gui Qt5::Widgets)

class_loader_hide_library_symbols(csapex_rqt)



#
# INSTALL
#

install(FILES rqt_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS csapex_rqt
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
