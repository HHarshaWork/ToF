cmake_minimum_required(VERSION 3.0)
project(examples)

add_subdirectory(first-frame)
add_subdirectory(data_collect)
if (NOT NXP)
    ################Setting up opencv and tensorflow for example code ########################
    set(OPENCV_URL "https://github.com/opencv/opencv/releases/download/4.5.3/opencv-4.5.3-openvino-dldt-2021.4-vc16-avx2.zip")
    set(OPENCV_DOWNLOAD_PATH "${CMAKE_SOURCE_DIR}/examples/people-detect/Dependencies/opencv-4.5.3-openvino-dldt-2021.4-vc16-avx2.zip")
    set(OPENCV_EXTRACTED_FILE "${CMAKE_SOURCE_DIR}/examples/people-detect/Dependencies/opencv/")

    set(TF_URL "https://storage.googleapis.com/tensorflow/libtensorflow/libtensorflow-cpu-windows-x86_64-1.15.0.zip")
    set(TF_DOWNLOAD_PATH "${CMAKE_SOURCE_DIR}/examples/people-detect/Dependencies/libtensorflow-cpu-windows-x86_64-1.15.0.zip")
    set(TF_EXTRACTED_FILE "${CMAKE_SOURCE_DIR}/examples/people-detect/Dependencies/tensorflow/")

    if (NOT EXISTS "${OPENCV_DOWNLOAD_PATH}")
        MESSAGE( STATUS "Downloading OpenCV")
        file(DOWNLOAD "${OPENCV_URL}" "${OPENCV_DOWNLOAD_PATH}")
        
    endif()

    if (NOT EXISTS "${TF_DOWNLOAD_PATH}")
        MESSAGE( STATUS "Downloading Tensorflow")
        file(DOWNLOAD "${TF_URL}" "${TF_DOWNLOAD_PATH}")
        
    endif()

    if (NOT EXISTS "${TF_EXTRACTED_FILE}")
        MESSAGE( STATUS "Unzipping Tensorflow")
        file(ARCHIVE_EXTRACT INPUT ${TF_DOWNLOAD_PATH} DESTINATION ${TF_EXTRACTED_FILE})
    endif()

    if (NOT EXISTS "${OPENCV_EXTRACTED_FILE}")
        MESSAGE( STATUS "Unzipping OpenCV")
        file(ARCHIVE_EXTRACT INPUT ${OPENCV_DOWNLOAD_PATH} DESTINATION ${OPENCV_EXTRACTED_FILE})
    endif()
    ################Setting up opencv and tensorflow for example code########################
    add_subdirectory(tof-viewer)
    add_subdirectory(people-detect)
endif()

if (WITH_NETWORK)
    add_subdirectory(first-frame-network)
endif()

include(GNUInstallDirs)

if (NXP)
    install(PROGRAMS
        $<TARGET_FILE:first-frame>
        $<TARGET_FILE:data_collect>
        DESTINATION ${CMAKE_INSTALL_BINDIR} )
elseif(WIN32)
    install(PROGRAMS
        $<TARGET_FILE:first-frame>
        $<TARGET_FILE:data_collect>
        $<TARGET_FILE:ADIToFGUI>
        DESTINATION ${CMAKE_INSTALL_BINDIR} )
endif()
