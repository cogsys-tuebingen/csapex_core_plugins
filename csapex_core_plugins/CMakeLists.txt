cmake_minimum_required(VERSION 2.8.3)
project(csapex_core_plugins)

find_package(catkin REQUIRED COMPONENTS csapex)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_add_resources(QT_RESOURCES
    res/csapex_core_resources.qrc
)

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
  LIBRARIES csapex_core_lib
  CATKIN_DEPENDS csapex roscpp
#  DEPENDS
)

###########
## Build ##
###########

include_directories(include
    ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_library(csapex_core_lib
    src/core/composite_message.cpp
    src/core/point_message.cpp
    src/core/timestamp_message.cpp
    src/core/duration_message.cpp
    src/core/interactive_node.cpp
    src/core/collection_node.cpp
    src/core/vector_process_node.cpp

    ${QT_RESOURCES}
)

target_link_libraries(csapex_core_lib
    ${catkin_LIBRARIES}
)

add_library(csapex_core_plugins_core
    src/core/register_core_plugins.cpp

    src/core/file_drag_io.cpp
)

target_link_libraries(csapex_core_plugins_core csapex_core_lib
    ${catkin_LIBRARIES}
)



add_library(csapex_core_plugins_node

    src/io/export_cout.cpp
    src/io/export_file.cpp
    src/io/file_importer.cpp
    src/io/import_cin.cpp
    src/io/import_file.cpp

    src/math/double_input.cpp
    src/math/statistics.cpp

    src/text/say_text.cpp
    src/text/text_convert.cpp
    src/text/text_input.cpp

    src/tools/cache.cpp
    src/tools/calculate_duration.cpp
    src/tools/composer.cpp
    src/tools/decomposer.cpp
    src/tools/delay.cpp
    src/tools/duration_convertion.cpp
    src/tools/holdable_buffer.cpp
    src/tools/relay.cpp
    src/tools/shutdown.cpp
    src/tools/signal_delay_measurement.cpp
    src/tools/stamp_delay_measurement.cpp
    src/tools/synchronizer.cpp
    src/tools/vector_merge.cpp

    src/display/output_display.cpp
    src/display/text_display.cpp
    src/display/signal_light.cpp
)

target_link_libraries(csapex_core_plugins_node csapex_core_lib
    ${catkin_LIBRARIES}
)



add_library(csapex_core_plugins_qt
    src/display/output_display_adapter.cpp
    src/display/text_display_adapter.cpp
    src/display/signal_light_adapter.cpp
)


target_link_libraries(csapex_core_plugins_qt csapex_core_lib
    ${catkin_LIBRARIES}
)


#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS csapex_core_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_core_plugins_core
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_core_plugins_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_core_plugins_qt
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

