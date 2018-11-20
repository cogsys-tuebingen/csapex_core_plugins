# Copyright (C) 2007 by Cristóbal Carnero Liñán
# grendel.ccl@gmail.com
#
# This file is part of cvBlob.
#
# cvBlob is free software: you can redistribute it and/or modify
# it under the terms of the Lesser GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# cvBlob is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# Lesser GNU General Public License for more details.
#
# You should have received a copy of the Lesser GNU General Public License
# along with cvBlob.  If not, see <http://www.gnu.org/licenses/>.
#

include (GenerateExportHeader)

find_package(OpenCV REQUIRED)

add_definitions(-w)

set(cvBlob_CVBLOB cvblob.cpp
                   cvlabel.cpp
		   cvaux.cpp
		   cvcontour.cpp
		   cvtrack.cpp
		   cvcolor.cpp
)

set_source_files_properties(${cvBlob_SRC}
                            PROPERTIES
                            COMPILE_FLAGS "-O2"
)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/..)

add_library(cvblob SHARED ${cvBlob_CVBLOB})
generate_export_header(cvblob)

target_link_libraries(cvblob ${OpenCV_LIBS})

install(TARGETS cvblob
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
		
if(WIN32)
	file(COPY ${CMAKE_CURRENT_BINARY_DIR}/cvblob_export.h
		 DESTINATION ${CMAKE_CURRENT_LIST_DIR}/../../include/cvBlob/)
	install(FILES ${PROJECT_BINARY_DIR}/src/cvBlob/cvblob_export.h
			DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/cvBlob/)
else()
	file(COPY ${CMAKE_CURRENT_BINARY_DIR}/cvblob_export.h
		 DESTINATION ${CMAKE_CURRENT_LIST_DIR}/../../include/cvBlob/)
	install(FILES ${PROJECT_BINARY_DIR}/src/cvBlob/cvblob_export.h
			DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
endif()
