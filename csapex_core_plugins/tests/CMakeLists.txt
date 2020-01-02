cmake_minimum_required(VERSION 3.8.2)

project(csapex_core_plugins_tests CXX)

enable_testing()

## Enforce that we use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#find_package(GTest REQUIRED)
include( CTest )

find_package(catkin REQUIRED COMPONENTS csapex_testing)

include_directories(${catkin_INCLUDE_DIRS} ${GTEST_INCLUDE_DIR} include)


# tests that are not yet distributed across the project
file(GLOB tests_SRC
    "src/*.cpp"
)
add_executable(${PROJECT_NAME}
    ${tests_SRC}
)
target_compile_definitions(${PROJECT_NAME}
    PRIVATE
    PACKAGE_XML="${CMAKE_CURRENT_LIST_DIR}/../plugins.xml"
    TEST_RES_DIR="${CMAKE_CURRENT_LIST_DIR}/res"
)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
set_tests_properties(${PROJECT_NAME} PROPERTIES TIMEOUT 60)
target_link_libraries(${PROJECT_NAME}
    csapex_core_plugins_core
    csapex_core_plugins_node
    ${catkin_LIBRARIES}
    gtest gtest_main)
