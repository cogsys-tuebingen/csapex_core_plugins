cmake_minimum_required(VERSION 2.8.3)
project(csapex_core_plugins)

if(NOT Boost_USE_STATIC_LIBS)
	set(Boost_USE_STATIC_LIBS TRUE)
endif()
if(NOT BOOST_ROOT)
	set(BOOST_ROOT "C:/boost_1_61_0")
endif()
if(NOT BOOST_LIBRARYDIR)
	set(BOOST_LIBRARYDIR "C:/boost_1_61_0/lib64-msvc-14.0")
endif()

set(OpenCV_DIR "C:/opencv/build")

add_subdirectory(csapex_core_plugins)
add_subdirectory(csapex_opencv)
add_subdirectory(csapex_windows)