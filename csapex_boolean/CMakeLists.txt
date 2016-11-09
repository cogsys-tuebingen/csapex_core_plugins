cmake_minimum_required(VERSION 2.8.3)
project(csapex_boolean)

find_package(csapex REQUIRED)

csapex_package()

include_directories(
    ${csapex_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_library(csapex_boolean_plugin_core SHARED
    src/register_plugin.cpp
)
target_link_libraries(csapex_boolean_plugin_core
    ${catkin_LIBRARIES}
	${csapex_LIBRARIES}
	)


add_library(csapex_boolean_plugin_node SHARED
    src/threshold.cpp

    src/nand.cpp
    src/toggle.cpp
    src/indicator.cpp

    res/csapex_boolean_resources.qrc
)
target_link_libraries(csapex_boolean_plugin_node
	${catkin_LIBRARIES}
	${csapex_LIBRARIES}
	)

#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS csapex_boolean_plugin_core
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS csapex_boolean_plugin_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

