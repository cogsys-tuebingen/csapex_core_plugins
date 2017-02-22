cmake_minimum_required(VERSION 2.8.3)
project(csapex_arff)

include (GenerateExportHeader)

list(APPEND CMAKE_PREFIX_PATH ${CSAPEX_CMAKE_MODULE_PATHS})
list(APPEND CMAKE_MODULE_PATH ${CSAPEX_CMAKE_MODULE_PATHS})

find_package(catkin REQUIRED COMPONENTS
        csapex
        csapex_ml
        csapex_core_plugins
        cslibs_arff
)


csapex_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS csapex_ml csapex_core_plugins cslibs_arff
    )

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(
    include
    ${csapex_INCLUDE_DIRS}
    ${csapex_core_plugins_INCLUDE_DIRS}
    ${csapex_ml_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUED_DIRS}
    ${cslibs_arff_INCLUDE_DIRS}    
)
add_library(${PROJECT_NAME}_plugin SHARED
        src/arff_file_exporter.cpp
        src/arff_feature_message_provider.cpp
	)

target_link_libraries(${PROJECT_NAME}_plugin
    ${PROJECT_NAME}
    ${csapex_LIBRARIES}
    ${csapex_core_plugins_LIBRARIES}
    ${csapex_ml_LIBRARIES}
    ${catkin_LIBRARIES}
    ${cslibs_arff_LIBRARIES}
    )

#
# INSTALL
#

install(FILES plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
