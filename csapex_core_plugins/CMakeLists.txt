cmake_minimum_required(VERSION 2.8.3)
project(csapex_core_plugins)

find_package(catkin REQUIRED COMPONENTS csapex)

QT4_ADD_RESOURCES(QT_RESOURCES res/csapex_core_resources.qrc)

csapex_moc_all(MOC_CORE include/*.h)
csapex_moc_all(MOC_PLUG src/*.h)

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
    ${catkin_INCLUDE_DIRS}
)

add_library(csapex_core_lib
    src/vector_message.cpp
    src/point_message.cpp
    src/interactive_node.cpp
    src/collection_node.cpp
    ${QT_RESOURCES}
    ${MOC_CORE}
)

target_link_libraries(csapex_core_lib
    ${catkin_LIBRARIES}
)

add_library(csapex_core_plugins
    src/import_file.cpp
    src/double_buffer.cpp
    src/relay.cpp
    src/delay.cpp
    src/holdable_buffer.cpp
    src/export_cout.cpp
    src/export_file.cpp
    src/file_importer.cpp
    src/import_cin.cpp
    src/double_input.cpp
    src/text_input.cpp

    src/text_display.cpp
    src/text_display_adapter.cpp

    src/say_text.cpp
    src/text_convert.cpp
    src/register_core_plugins.cpp
    ${QT_RESOURCES}
    ${MOC_PLUG}
)

target_link_libraries(csapex_core_plugins csapex_core_lib
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

install(TARGETS csapex_core_plugins
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

