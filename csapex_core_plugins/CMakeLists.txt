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
    src/vector_message.cpp
    src/composite_message.cpp
    src/point_message.cpp
    src/interactive_node.cpp
    src/collection_node.cpp

    ${QT_RESOURCES}
)

target_link_libraries(csapex_core_lib
    ${catkin_LIBRARIES}
)

add_library(csapex_core_plugins_core
    src/register_core_plugins.cpp
)

target_link_libraries(csapex_core_plugins_core csapex_core_lib
    ${catkin_LIBRARIES}
)



add_library(csapex_core_plugins_node
    src/import_file.cpp
    src/relay.cpp
    src/delay.cpp
    src/holdable_buffer.cpp
    src/export_cout.cpp
    src/export_file.cpp
    src/file_importer.cpp
    src/import_cin.cpp
    src/double_input.cpp
    src/text_input.cpp

    src/stamp_delay_measurement.cpp
    src/signal_delay_measurement.cpp

    src/demultiplexer.cpp
    src/multiplexer.cpp

    src/vector_merge.cpp

    src/composer.cpp
    src/decomposer.cpp

    src/output_display.cpp
    src/text_display.cpp

    src/say_text.cpp
    src/text_convert.cpp
)

target_link_libraries(csapex_core_plugins_node csapex_core_lib
    ${catkin_LIBRARIES}
)



add_library(csapex_core_plugins_qt
    src/file_drag_io.cpp
    src/output_display_adapter.cpp
    src/text_display_adapter.cpp
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

