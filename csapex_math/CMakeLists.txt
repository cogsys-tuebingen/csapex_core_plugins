cmake_minimum_required(VERSION 2.8.3)
project(csapex_math)

find_package(csapex REQUIRED)

find_package(Qt5 COMPONENTS Core REQUIRED)

csapex_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_core
    CATKIN_DEPENDS csapex
)

include_directories(
    include
    ${csapex_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

set(CMAKE_AUTOMOC ON)

qt5_add_resources(QT_RESOURCES res/csapex_math_resources.qrc)


add_library(${PROJECT_NAME} SHARED
    src/param/angle_parameter.cpp
    src/param/factory.cpp
    #src/param/vector_parameter.cpp
)
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${csapex_LIBRARIES}
)


add_library(${PROJECT_NAME}_core SHARED
    src/register_math.cpp
)
target_link_libraries(${PROJECT_NAME}_core
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${csapex_LIBRARIES}
)



add_library(${PROJECT_NAME}_qt SHARED
    #src/view/vector_param_adapter.cpp
    src/view/angle_param_adapter.cpp
    ${QT_RESOURCES}
)
target_link_libraries(${PROJECT_NAME}_qt
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${csapex_LIBRARIES}
)



add_library(${PROJECT_NAME}_node SHARED
    src/norm_of_generic_vector_message.cpp
    src/invert_values.cpp
    src/multiply_values.cpp
    src/generic_vector_message_get_extremum.cpp
    src/scale_generic_vector_message.cpp
    src/negate_number.cpp
    src/sum.cpp

    ${QT_RESOURCES}
)
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${csapex_LIBRARIES}
)

#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ${PROJECT_NAME}_core
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ${PROJECT_NAME}_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ${PROJECT_NAME}_qt
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})





