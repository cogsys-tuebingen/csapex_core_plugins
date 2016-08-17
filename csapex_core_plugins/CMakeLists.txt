cmake_minimum_required(VERSION 2.8.3)
project(csapex_core_plugins)

include (GenerateExportHeader)

find_package(csapex REQUIRED)

find_package(catkin QUIET)

if(${catkin_FOUND})
	catkin_package(
	  INCLUDE_DIRS include
	  LIBRARIES csapex_core_lib
	  CATKIN_DEPENDS csapex roscpp
	)

else()
	find_package(YAML-CPP REQUIRED)
	find_package(console_bridge REQUIRED)
	find_package(class_loader REQUIRED)
	if(WIN32)
		add_definitions(/D POCO_NO_AUTOMATIC_LIBS)
		add_definitions(/D NOMINMAX)
	endif()
	
	find_package(Boost COMPONENTS program_options filesystem system regex serialization thread REQUIRED)

	set(INSTALL_DIR ${CMAKE_INSTALL_PREFIX})
	set(CATKIN_PACKAGE_INCLUDE_DESTINATION ${INSTALL_DIR}/include)
	set(CATKIN_PACKAGE_LIB_DESTINATION ${INSTALL_DIR}/lib)
	set(CATKIN_GLOBAL_BIN_DESTINATION ${INSTALL_DIR}/bin)
	set(CATKIN_PACKAGE_SHARE_DESTINATION ${INSTALL_DIR}/share/${PROJECT_NAME})
	
	set(catkin_INCLUDE_DIRS
		${Boost_INCLUDE_DIRS} 
		${YAML_CPP_INCLUDE_DIR})
	set(catkin_LIBRARIES 
		${Boost_LIBRARIES}
		${YAML_CPP_LIBRARIES}
		${class_loader_LIBRARIES}
		${console_bridge_LIBRARIES}
		${Poco_LIBRARIES})
endif()

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_add_resources(QT_RESOURCES
    res/csapex_core_resources.qrc
)


###########
## Build ##
###########


include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(include
    ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
    ${csapex_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

add_library(csapex_core_lib SHARED
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
    ${csapex_LIBRARIES}
    Qt5::Core
)

add_library(csapex_core_plugins_core SHARED
    src/core/register_core_plugins.cpp

    src/core/file_drag_io.cpp
)


target_link_libraries(csapex_core_plugins_core csapex_core_lib
    ${catkin_LIBRARIES}
    ${csapex_LIBRARIES}
    Qt5::Core Qt5::Gui 
)



add_library(csapex_core_plugins_node SHARED

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
    src/tools/select_vector_component.cpp

    src/display/output_display.cpp
    src/display/text_display.cpp
    src/display/signal_light.cpp
)

target_link_libraries(csapex_core_plugins_node csapex_core_lib
    ${catkin_LIBRARIES}
    ${csapex_LIBRARIES}
    Qt5::Core Qt5::Gui Qt5::Widgets
)

generate_export_header(csapex_core_plugins_node)


add_library(csapex_core_plugins_qt SHARED
    src/display/output_display_adapter.cpp
    src/display/text_display_adapter.cpp
    src/display/signal_light_adapter.cpp
)


target_link_libraries(csapex_core_plugins_qt 
	csapex_core_plugins_node
	csapex_core_lib

    ${catkin_LIBRARIES}
    ${csapex_LIBRARIES}
    Qt5::Core Qt5::Gui Qt5::Widgets
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

if(WIN32)
	install(DIRECTORY include/${PROJECT_NAME}/
			DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${PROJECT_NAME})
else()
	install(DIRECTORY include/${PROJECT_NAME}/
			LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
			DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
endif()