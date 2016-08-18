include(${CMAKE_INSTALL_PREFIX}/CMake/Findcsapex_plugin.cmake)

find_csapex_plugin(csapex_opencv csapex_opencv cv_mat_message.h)

find_package(OpenCV REQUIRED)
