cmake_minimum_required(VERSION 2.8.11)
project(csapex_optimization)

find_package(catkin REQUIRED COMPONENTS
  csapex
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES csapex_optimization
  CATKIN_DEPENDS csapex
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(csapex_optimization
    src/parameter_dialog.cpp
    include/csapex_optimization/parameter_dialog.h
)
target_link_libraries(csapex_optimization
    yaml-cpp ${Boost_LIBRARIES} ${catkin_LIBRARIES}
)


add_library(csapex_optimization_plugin
    src/parameter_dialog.cpp
    src/optimization_dummy.cpp
    src/bf_optimizer.cpp
    src/bf_optimizer_adapter.cpp
)

target_link_libraries(csapex_optimization_plugin
    yaml-cpp csapex_optimization ${catkin_LIBRARIES})

#
# INSTALL
#

install(FILES plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS csapex_optimization
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
        
install(TARGETS csapex_optimization_plugin
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
