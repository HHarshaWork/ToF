# Specify the minimum version for CMake

cmake_minimum_required(VERSION 2.8)

# Project's name
project(PeopleDetectApplication)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# adding submodules and subdirectories -- if needed to build with CMake
# add_subdirectory(external/opencv)
# add_subdirectory(external/tof)


if( WIN32 )
    set(OS_SPECIFIC_DIR windows)
elseif ( APPLE )
    set(CMAKE_CXX_FLAGS "-x objective-c")
    set(OS_SPECIFIC_SOURCES ${ADIToF_SOURCE_DIR}/fileDialog.mm) 	
elseif ( UNIX )
    set(OS_SPECIFIC_DIR linux)
else()
    message(FATAL_ERROR "Platform not supported")
endif()

set(ADIToF_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Application)
#set(LIB_DIR ${PROJECT_SOURCE_DIR}/Dependencies)

# add_custom_command(
    # OUTPUT "${MY_EXTRACTED_FILE}"
    # COMMAND unzip
    # DEPENDS "${MY_DOWNLOAD_PATH}")

add_executable(${PROJECT_NAME} 
                ${ADIToF_SOURCE_DIR}/src/PeopleDetectionApplication.cpp
                ${ADIToF_SOURCE_DIR}/src/FileIO.cpp
                ${ADIToF_SOURCE_DIR}/src/ParseArgs.cpp
                ${ADIToF_SOURCE_DIR}/src/ParseCmd.cpp
                ${ADIToF_SOURCE_DIR}/src/SendHeadLocations.cpp
                )

# Link Libraries
# find_package(glog REQUIRED)
# add_library(glog SHARED IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/Dependencies/glog)
# add_library(opencv SHARED IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/Dependencies/opencv)

target_link_libraries(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/../../deps/glog/build_0_3_5/local_path/glog/lib/glog.lib)
target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/Lib/ObjectDetector/ObjectDetector.lib)
target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/Lib/PeopleDetector/PeopleDetection.lib)
target_link_libraries(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/../../build/sdk/Release/aditof.lib)
target_link_libraries(${PROJECT_NAME} 
						"${PROJECT_SOURCE_DIR}/Dependencies/opencv/opencv/build/lib/opencv_core453.lib"
						"${PROJECT_SOURCE_DIR}/Dependencies/opencv/opencv/build/lib/opencv_imgproc453.lib"
						"${PROJECT_SOURCE_DIR}/Dependencies/opencv/opencv/build/lib/opencv_highgui453.lib"
						"${PROJECT_SOURCE_DIR}/Dependencies/opencv/opencv/build/lib/opencv_imgcodecs453.lib"
						"${PROJECT_SOURCE_DIR}/Dependencies/opencv/opencv/build/lib/opencv_video453.lib"
						"${PROJECT_SOURCE_DIR}/Dependencies/opencv/opencv/build/lib/opencv_videoio453.lib"
	)

#Add include paths
target_include_directories(${PROJECT_NAME} PUBLIC
						"${PROJECT_BINARY_DIR}"
						"${PROJECT_SOURCE_DIR}/include"
                        "${PROJECT_SOURCE_DIR}/include/PeopleDetector"
                        "${PROJECT_SOURCE_DIR}/include/ObjectDetector"
                        "${ADIToF_SOURCE_DIR}/include"
						"${CMAKE_CURRENT_SOURCE_DIR}/../../deps/glog/build_0_3_5/local_path/glog/include"
                        "${PROJECT_SOURCE_DIR}/Dependencies/opencv/opencv/build/include"
						"${PROJECT_SOURCE_DIR}/include"
				)

target_include_directories(${PROJECT_NAME} PUBLIC
                        "${CMAKE_SOURCE_DIR}/sdk/include")
                        
                        
# # Set the output folder where your program will be created
# set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
# set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
# set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# Copying necessary dlls from SDK and the necessary config files
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy ${LIBTOFI_LIBDIR_PATH}/tofi_compute.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
	COMMAND ${CMAKE_COMMAND} -E copy ${LIBTOFI_LIBDIR_PATH}/tofi_config.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/../../build/sdk/Release/aditof.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
	COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/Lib/ObjectDetector/ObjectDetector.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
	COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/Lib/PeopleDetector/PeopleDetection.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>/.	
	COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/Dependencies/opencv/opencv/build/bin/opencv_core453.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
	COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/Dependencies/opencv/opencv/build/bin/opencv_imgproc453.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
	COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/Dependencies/opencv/opencv/build/bin/opencv_highgui453.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
	COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/Dependencies/opencv/opencv/build/bin/opencv_imgcodecs453.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
	COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/Dependencies/opencv/opencv/build/bin/opencv_video453.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
	COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/Dependencies/opencv/opencv/build/bin/opencv_videoio453.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
	COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/Dependencies/tensorflow/lib/tensorflow.dll $<TARGET_FILE_DIR:${PROJECT_NAME}>/.
	COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/src/cameras/itof-camera/config $<TARGET_FILE_DIR:${PROJECT_NAME}>/${CONFIG_DIR_NAME}/.
    )

# if(WIN32)
	# # copy sdk lib to exe directory
	# add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
					# COMMAND ${CMAKE_COMMAND} -E copy_directory $<TARGET_LINKER_FILE_DIR:aditof> ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>
					# VERBATIM
					# COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/src/cameras/itof-camera/config $<TARGET_FILE_DIR:${PROJECT_NAME}>/../${CONFIG_DIR_NAME}/.
					# )
	# if (WITH_NETWORK)
		# add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
		# COMMAND ${CMAKE_COMMAND} -E copy_directory ${LWS_CMAKE_DIR}/../bin ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>
		# )
	# endif()
# endif()
