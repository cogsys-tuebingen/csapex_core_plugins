cmake_minimum_required(VERSION 3.8.2)
project(csapex_tutorial CXX)

find_package(catkin REQUIRED COMPONENTS
    csapex_opencv
    csapex_ros
    message_generation
)

find_package(csapex REQUIRED)

# Generate ROS messages
add_message_files(
    FILES
    TutorialMessage.msg
)

generate_messages()

# Declare ROS package
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS message_runtime
)

# Define the includes
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


# A core library that contains an apex wrapper for the ROS Message
add_library(${PROJECT_NAME}
    src/tutorial_message.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${csapex_LIBRARIES}
    ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# A library to contain csapex::Node plugins
add_library(${PROJECT_NAME}_node
    src/user_switch.cpp
)
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

# A library to contain csapex::CorePlugin plugins
add_library(${PROJECT_NAME}_core
    src/tutorial_core_plugin.cpp
)
target_link_libraries(${PROJECT_NAME}_core
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

# Install the generated libraries and the plugins.xml file
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS ${PROJECT_NAME}_core
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(FILES plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



