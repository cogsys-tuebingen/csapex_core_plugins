cmake_minimum_required(VERSION 2.8.3)
project(csapex_windows)

find_package(csapex REQUIRED)
find_package(class_loader REQUIRED)
find_package(console_bridge REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)

find_package(Boost COMPONENTS program_options filesystem system regex serialization thread REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

add_definitions(/D POCO_NO_AUTOMATIC_LIBS)

include_directories(include 
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS}
    ${csapex_INCLUDE_DIR}
	${Boost_INCLUDE_DIRS}
)

set(INSTALL_DIR ${CMAKE_INSTALL_PREFIX})
set(CATKIN_PACKAGE_INCLUDE_DESTINATION ${INSTALL_DIR}/include)
set(CATKIN_PACKAGE_LIB_DESTINATION ${INSTALL_DIR}/lib)
set(CATKIN_GLOBAL_BIN_DESTINATION ${INSTALL_DIR}/bin/csapex_plugins)
set(CATKIN_PACKAGE_SHARE_DESTINATION ${INSTALL_DIR}/share)

## booting
add_library(csapex_windows_boot SHARED
    src/windows_boot.cpp
)
target_link_libraries(csapex_windows_boot
    #${catkin_LIBRARIES}
    ${csapex_LIBRARIES}
    ${Boost_LIBRARIES}
    ${class_loader_LIBRARIES}
    ${console_bridge_LIBRARIES}
    ${Poco_LIBRARIES}
)

#add_custom_command(TARGET csapex_windows_boot POST_BUILD
#    COMMAND mkdir -p ${CSAPEX_BOOT_PLUGIN_DIR} && ln -sf ${CATKIN_DEVEL_PREFIX}/lib/libcsapex_windows_boot.dll ${CSAPEX_BOOT_PLUGIN_DIR}/libcsapex_windows_boot.dll)


#
# INSTALL
#

install(TARGETS csapex_windows_boot
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})