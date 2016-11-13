cmake_minimum_required(VERSION 2.8.3)
project(csapex_state)

find_package(csapex REQUIRED)

find_package(Qt5 COMPONENTS Core REQUIRED)

csapex_package()

include_directories(
    ${csapex_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

set(CMAKE_AUTOMOC ON)

qt5_add_resources(QT_RESOURCES res/csapex_state_resources.qrc)

add_library(${PROJECT_NAME}_node SHARED
    src/inactivity_generator.cpp
    src/join.cpp
    src/switch.cpp
    src/activity_generator.cpp
    src/activity_absorber.cpp
    src/activity_switch.cpp
    src/activity_gate.cpp
    src/memory_cell.cpp

    ${QT_RESOURCES}
)
target_link_libraries(${PROJECT_NAME}_node
	${catkin_LIBRARIES}
	${csapex_LIBRARIES}
)

#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

