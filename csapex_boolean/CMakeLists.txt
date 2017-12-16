cmake_minimum_required(VERSION 2.8.3)
project(csapex_boolean)

find_package(csapex REQUIRED)

csapex_package()

include_directories(
    ${csapex_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_plugin_core SHARED
    src/register_plugin.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin_core
    ${catkin_LIBRARIES}
	${csapex_LIBRARIES}
	)


add_library(${PROJECT_NAME}_plugin_node SHARED
    src/threshold.cpp

    src/nand.cpp
    src/toggle.cpp
    src/indicator.cpp

    res/${PROJECT_NAME}_resources.qrc
)
target_link_libraries(${PROJECT_NAME}_plugin_node
	${catkin_LIBRARIES}
        ${csapex_LIBRARIES}
	)

#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_plugin_core ${PROJECT_NAME}_plugin_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

