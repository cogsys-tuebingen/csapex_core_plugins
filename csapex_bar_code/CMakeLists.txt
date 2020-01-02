cmake_minimum_required(VERSION 3.8.2)
project(csapex_bar_code CXX)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(ZBar REQUIRED)

find_package(csapex REQUIRED)

find_package(csapex_opencv REQUIRED)

csapex_package()

###########
## Build ##
###########

if(${ZBAR_FOUND})
    include_directories(
      ${csapex_INCLUDE_DIRS}
      ${csapex_opencv_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
      ${ZBAR_INCLUDE_DIR}
    )

	file(GLOB_RECURSE plugin_HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.h*)
	
    add_library(${PROJECT_NAME}_plugin SHARED
		${plugin_HEADERS}
		
        src/bar_code_reader.cpp
    )
	
    target_link_libraries(${PROJECT_NAME}_plugin
                ${csapex_LIBRARIES}
		${csapex_opencv_LIBRARIES}
		${catkin_LIBRARIES}
		${ZBAR_LIBRARIES}
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


endif(${ZBAR_FOUND})
