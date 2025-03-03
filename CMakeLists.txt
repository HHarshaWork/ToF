cmake_minimum_required(VERSION 3.0)
project(adi_tof_project)

####################### Disable In-source builds ##############################
if( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR} )
    message(FATAL_ERROR "In-source builds are not allowed. \
    Consider making a separate build folder and run cmake \
    from there using this command:
    cmake ${CMAKE_SOURCE_DIR}")
endif()

############################### Version #######################################
set(ADITOF_VERSION_MAJOR 3)
set(ADITOF_VERSION_MINOR 2)
set(ADITOF_VERSION_PATCH 0)

set(VERSION "${ADITOF_VERSION_MAJOR}.${ADITOF_VERSION_MINOR}.${ADITOF_VERSION_PATCH}")

############################### Options #######################################
option(WITH_EXAMPLES "Build examples?" ON)
option(WITH_DOC "Build documentation?" OFF)
option(WITH_PYTHON "Build python bindings?" OFF)
option(WITH_OPENCV "Build opencv bindings?" OFF)
option(WITH_OPEN3D "Build open3d bindings?" OFF)
option(WITH_ROS "Build ros bindings?" OFF)
option(WITH_NETWORK "Build network interface?" OFF)
option(WITH_OFFLINE "Build offline interface?" OFF)
option(USE_DEPTH_COMPUTE_STUBS "Use empty generated libs?" OFF)

################################## Git ########################################
include(FindGit OPTIONAL)
if (GIT_FOUND)
	execute_process(
		COMMAND ${GIT_EXECUTABLE} rev-parse --show-toplevel
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		OUTPUT_VARIABLE ADITOFSDK_GIT_REPO
		OUTPUT_STRIP_TRAILING_WHITESPACE
		ERROR_QUIET
	)

	if ("${ADITOFSDK_GIT_REPO}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
		execute_process(
			COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
			OUTPUT_VARIABLE ADITOFSDK_GIT_COMMIT
			OUTPUT_STRIP_TRAILING_WHITESPACE
		)
		execute_process(
			COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
			OUTPUT_VARIABLE ADITOFSDK_GIT_BRANCH
			OUTPUT_STRIP_TRAILING_WHITESPACE
		)
	endif()
endif()

add_definitions(-DADITOFSDK_GIT_COMMIT="${ADITOFSDK_GIT_COMMIT}")
add_definitions(-DADITOFSDK_GIT_BRANCH="${ADITOFSDK_GIT_BRANCH}")

############################## Rest of cmake ##################################
	
if (WITH_NETWORK)
        add_definitions(-DHAS_NETWORK)
endif()

set(RESOURCES_DIR "${CMAKE_BINARY_DIR}/resources")
make_directory(${RESOURCES_DIR})

if (WITH_OFFLINE)
        add_definitions(-DHAS_OFFLINE)

        set(RESOURCES_OFFLINE_DIR "${RESOURCES_DIR}/offline")
        make_directory(${RESOURCES_OFFLINE_DIR})
        
        set(MODE3_FILE "mode3_raw.bin")
        set(MODE3_URL "swdownloads.analog.com/cse/aditof/resources/itof/frames/${MODE3_FILE}")
        set(MODE3_PATH "${RESOURCES_OFFLINE_DIR}/${MODE3_FILE}")

        message("Downloading raw frames into ${RESOURCES_OFFLINE_DIR}")
        if (NOT EXISTS ${MODE3_PATH})
                file(DOWNLOAD "${MODE3_URL}" "${MODE3_PATH}")
        endif()
endif()

set(LIBTOFI_LIBDIR_PATH "${CMAKE_SOURCE_DIR}/libs")


set(CONFIG_DIR_NAME "config")
add_definitions(-DCONFIG_DIR_NAME="${CONFIG_DIR_NAME}")
add_definitions(-DRESOURCES="${RESOURCES_DIR}")

add_subdirectory(apps)
add_subdirectory(sdk)


if(WITH_TOOLS)
        # if (MSVC)
        #         add_subdirectory(tools/deps/wingetopt)
        # endif(MSVC)
        # add_subdirectory(tools/eeprom-tool)
endif()
if (WITH_EXAMPLES)
        add_subdirectory(examples)
endif()
if (WITH_DOC)
        add_subdirectory(doc)
endif()
if (WITH_PYTHON)
        add_subdirectory(bindings/python)
endif()
if (WITH_OPENCV)
        add_subdirectory(bindings/opencv)
endif()
if (WITH_OPEN3D)
        add_subdirectory(bindings/open3D)
endif()
if (WITH_ROS)
        add_subdirectory(bindings/ros)
endif()

############################### Installer #######################################
configure_file(cmake/aditof-setup.iss.cmakein ${CMAKE_CURRENT_BINARY_DIR}/aditof-setup.iss @ONLY)

############################### Version #######################################
configure_file(cmake/version.h.cmakein ${CMAKE_SOURCE_DIR}/sdk/include/aditof/version.h @ONLY)
