/******************************************************************************
 Copyright(c) 2022 Analog Devices, Inc. All Rights Reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.  By using this software you agree
 to the terms of the associated Analog Devices Software License Agreement.
 *******************************************************************************/
/*******************************************************************************
 @file :FileIO.cpp

 @brief :Functions for reading and writing files and input streams

 *********************************************************************************/

#include "FileIO.h"
#include <string>
#include <iostream>

//OpenCV is used for writing output frame to AVI file
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
using namespace cv;
using namespace std;

/**
 * Constructor for fileio class
 *
 */
FileIO::FileIO()
{
	_fpIn = NULL;
	_fpMetaData = NULL;
    _fpAVIWriter = NULL;

	_nInputFrameSize = 0;
	_nFrameCounter = 0;
    _nOutWidth = 1024;
    _nOutHeight = 1024;

	_camera = NULL;
	_file_io_mode = true;
}

/**
 * Constructor for fileio class
 *
 * @param width - width of image
 * @param height - height of image
 * @return None
 */
FileIO::FileIO(int width, int height)
{
	_fpIn = NULL;
	_fpMetaData = NULL;
	_fpAVIWriter = NULL;

	_nInputFrameSize = 0;
	_nFrameCounter = 0;

	//Output file in WIN32 mode will always be in QVGA.
	_nOutWidth = 1024;
	_nOutHeight = 1024;

	_nOutWidth = width;
	_nOutHeight = height;

	_nInputFrameSize = (_nOutWidth * _nOutHeight * 2 * sizeof(uint16_t));

	_camera = NULL;
	_file_io_mode = true;
}

/**
 * Opens input file
 *
 * @param pInFileName - Input filename
 * @param nMode - Capture mode 1-512x512, 0-1024x1024
 * @return 1 -  Success , 0 Failure 
 */
FILE_IO_STATUS FileIO::OpenInputFile(char *pInFileName, int nMode)
{
	std::string extension;
	printf("Opening input file: %s\n", pInFileName);

	if (strcmp(pInFileName, "camera") == 0)
	{
		//FLAGS_alsologtostderr = 1;

		//realtime mode
		Status status = Status::OK;
		System system;

		std::vector<std::shared_ptr<Camera>> cameras;		
		system.getCameraList(cameras);
		if (cameras.empty()) {
			cout << "No cameras found";
			return FILE_IO_FAILURE;
		}

		_camera = cameras.front();

		// user can pass any config.json stored anywhere in HW
		status = _camera->setControl("initialization_config", "config.json");
		if (status != Status::OK) {
			cout << "Could not set the initialization config file!";
			return 0;
		}

		status = _camera->initialize();
		if (status != Status::OK) {
			cout << "Could not initialize camera!";
			return FILE_IO_FAILURE;
		}

		status = _camera->setControl("powerUp", "call");
		if (status != Status::OK) {
			cout << "Could not PowerUp camera!";
			return 0;
		}

		// load configuration data from module memory
		status = _camera->setControl("loadModuleData", "call");
		if (status != Status::OK) {
			cout << "No CCB/CFG data found in camera module,";
		}

		std::vector<std::string> frameTypes;
		_camera->getAvailableFrameTypes(frameTypes);
		if (frameTypes.empty()) {
			std::cout << "no frame type avaialble!";
			return FILE_IO_FAILURE;
		}

		//enabled depth computation
		_camera->setControl("enableDepthCompute", "on");

		std::map<int, int> modeIndexMap = { {1, 0}, {3, 1}, {5, 2}, {7, 3}, {10, 4} };
		int nSetMode = 10;//1Mpixel (1024x1024)
		if (nMode == 1)
		{
			nSetMode = 7;//QMP (512x512)
		}
		status = _camera->setFrameType(frameTypes[modeIndexMap[nSetMode]]);
		if (status != Status::OK) {
			cout << "Could not set camera frame type!";
			return FILE_IO_FAILURE;
		}

		status = _camera->start();
		if (status != Status::OK) {
			cout << "Could not start the camera!";
			return FILE_IO_FAILURE;
		}

		_file_io_mode = false;
	}
	else
	{
		//fileio mode
		string FileName(pInFileName);
		int32_t idx = (int32_t)FileName.rfind('.');
		if (idx != std::string::npos)
		{
			extension = FileName.substr(idx + 1);
		}
		for (std::string::size_type i = 0; i < extension.length(); ++i)
		{
			extension[i] = std::tolower(extension[i]);
		}
		
		m_infile.open(pInFileName, ios::binary);
		if (!m_infile.is_open())
		{
			cout << "Error opening input file : %s" << pInFileName << endl;
			return FILE_IO_FAILURE;
		}		
		_file_io_mode = true;
	}
    return FILE_IO_SUCCESS;
}

/**
 * Opens output file
 *
 * @param pOutFileName - Output filename
  * @return 1 -  Success , 0 Failure
 */
FILE_IO_STATUS FileIO::OpenOutputFile(char *pOutFileName)
{
    std::string OutFileName = pOutFileName;

	printf("Opening output file: %s\n", pOutFileName);


    /* On WIndows it is always an AVI file */
	int nFrameRate = 1;
	VideoWriter* writer = new VideoWriter(pOutFileName, VideoWriter::fourcc('M', 'J', 'P', 'G'),
		nFrameRate, Size(_nOutWidth, _nOutHeight), true );
    if (!writer->isOpened())
    {
        cout << "Error: Opening AVI Video file to write " << endl;
        return FILE_IO_FAILURE;
    }
    _fpAVIWriter = (void*)writer;
    
	return FILE_IO_SUCCESS;
}

/**
 * Opens metadata file
 *
 * @param pMetaDataFileName - Metadata filename
 * @return 1 -  Success , 0 Failure
 */
FILE_IO_STATUS FileIO::OpenMetadataFile(char *pMetaDataFileName)
{
	printf("Opening metadata file: %s\n", pMetaDataFileName);

	_fpMetaData = fopen(pMetaDataFileName, "w");
    if (_fpMetaData == NULL)
    {
    	fprintf(stderr, "Error opening metadata file : %s\n", pMetaDataFileName);
        return FILE_IO_FAILURE;
    }

    fprintf(_fpMetaData, "Frame_Number, # Locations, Locations\n");

	return FILE_IO_SUCCESS;
}

/**
 * Inititializes input file
 *
 * @param pInFileName - Input filename
 * @param nMode - Capture mode 1-512x512, 0-1024x1024
 * @return 1 -  Success , 0 Failure
 */
FILE_IO_STATUS FileIO::Init(char *pInFileName, int nMode)
{
	FILE_IO_STATUS nStatus;

	_fpIn = NULL;
	_fpMetaData = NULL;
    _fpAVIWriter = NULL;

	_nFrameCounter = 0;

	nStatus = OpenInputFile(pInFileName,nMode);

	return nStatus;
}

/**
 * Inititializes input file and metadata file
 *
 * @param pInFileName - Input filename
 * @param pMetaDataFileName - Metadata filename* 
 * @param nMode - Capture mode 1-512x512, 0-1024x1024
 * @return 1 -  Success , 0 Failure
 */
FILE_IO_STATUS FileIO::Init(char *pInFileName, char *pMetaDataFileName, int nMode)
{
	FILE_IO_STATUS nStatus;

	_fpIn = NULL;
	_fpMetaData = NULL;
    _fpAVIWriter = NULL;

	_nFrameCounter = 0;

	nStatus = OpenInputFile(pInFileName, nMode);
	if (nStatus == FILE_IO_FAILURE)
	{
		return nStatus;
	}

	nStatus = OpenMetadataFile(pMetaDataFileName);
	if (nStatus == FILE_IO_FAILURE)
	{
		return nStatus;
	}

	return FILE_IO_SUCCESS;
}

/**
 * Inititializes input file, outputfile  and metadata file
 *
 * @param pInFileName - Input filename
 * @param pMetaDataFileName - Metadata filename
 * @param pOutFileName - Output filename
 * @param nMode - Capture mode 1-512x512, 0-1024x1024
 * @return 1 -  Success , 0 Failure
 */
FILE_IO_STATUS FileIO::Init(char *pInFileName, char *pMetaDataFileName, 
	char *pOutFileName, int nMode)
{
	FILE_IO_STATUS nStatus;

	_fpIn = NULL;
	_fpMetaData = NULL;
    _fpAVIWriter = NULL;

	_nFrameCounter = 0;

	nStatus = OpenInputFile(pInFileName, nMode);
	if (nStatus == FILE_IO_FAILURE)
	{
		return nStatus;
	}

	nStatus = OpenOutputFile(pOutFileName);
	if (nStatus == FILE_IO_FAILURE)
	{
		return nStatus;
	}

	nStatus = OpenMetadataFile(pMetaDataFileName);
	if (nStatus == FILE_IO_FAILURE)
	{
		return nStatus;
	}

	return FILE_IO_SUCCESS;
}

/**
 * Get framecount
 *
 * @return _nFrameCounter -  Num of read frames
 */
int32_t FileIO::GetFrameCounter()
{
	return _nFrameCounter;
}

/**
 * Set input read buffer frame size
 *
 * @param nWidth - Width
 * @param nHeight - Height
 * @return None
 */
void FileIO::SetInputFrameSize(uint32_t nWidth, uint32_t nHeight)
{
	//ir image and depth image of uint16_t
	_nOutWidth = nWidth;
	_nOutHeight = nHeight;

	//ir-1,depth-1,point-cloud-3
	_nInputFrameSize = (_nOutWidth * _nOutHeight * 5 * sizeof(uint16_t));
}

/**
 * Read input frame
 *
 * @param pBuf - Buffer to read frame in 
 * @return 1 -  Success , 0 Failure
 */
FILE_IO_STATUS FileIO::Read(uint8_t *pBuf)
{
    int32_t nFileSeekSet = 0;
	if (_file_io_mode == true)
	{
		m_infile.read((char*)pBuf, _nInputFrameSize);
		if (m_infile.gcount() != _nInputFrameSize)
		{
			return FILE_IO_FAILURE;
		}
	}
	else
	{

		aditof::Frame frame;

		Status status = _camera->requestFrame(&frame);
		if (status != Status::OK) {
			cout << "Could not request frame!";
			return FILE_IO_FAILURE;
		}		

		uint16_t* depthFrameLocation;
		status = frame.getData("depth", &depthFrameLocation);
		if (status != Status::OK) {
			cout << "Could not get depth data!";
			return FILE_IO_FAILURE;
		}
		memcpy(pBuf, depthFrameLocation, _nOutWidth * _nOutHeight * sizeof(uint16_t));
		pBuf += _nOutWidth * _nOutHeight * sizeof(uint16_t);

		uint16_t* irFrameLocation;
		status = frame.getData("ir", &irFrameLocation);
		if (status != Status::OK) {
			cout << "Could not get ir data!";
			return FILE_IO_FAILURE;
		}
		memcpy(pBuf, irFrameLocation, _nOutWidth * _nOutHeight * sizeof(uint16_t));
		pBuf += _nOutWidth * _nOutHeight * sizeof(uint16_t);

		uint16_t* xyzFrameLocation;
		status = frame.getData("xyz", &xyzFrameLocation);
		if (status != Status::OK) {
			cout << "Could not get ir data!";
			return FILE_IO_FAILURE;
		}
		memcpy(pBuf, xyzFrameLocation, _nOutWidth * _nOutHeight * 3 * sizeof(uint16_t));
		pBuf += _nOutWidth * _nOutHeight * 3 * sizeof(uint16_t);
	}
	
    _nFrameCounter += 1;
    return FILE_IO_SUCCESS;
}

/**
 * Write output frame
 *
 * @param pBuf - Buffer to write frame in
 * @param nNumBytesToWrite - Num of bytes to write
 * @return 1 -  Success , 0 Failure
 */
FILE_IO_STATUS FileIO::Write(uint8_t *pBuf, uint32_t nNumBytesToWrite)
{
    if (_fpAVIWriter != NULL)
    {
        VideoWriter* writer = (VideoWriter*)_fpAVIWriter;
		//load mat that is 
        Mat img(_nOutHeight, _nOutWidth, CV_8UC3);
        memcpy(img.data, pBuf, _nOutWidth*_nOutHeight*3);//rgb image of 8 bit
        writer->write(img);
    }

    return FILE_IO_SUCCESS;
}

/**
 * Write metadata output
 *
 * @param nFrameId - Frame number
 * @param nDetectedObjects - Num of detected objects'
 * @param oDetectedObjectsInfo - properties and location of detected objects 
 * @return 1 -  Success , 0 Failure
 */
void FileIO::WriteMetaData(int32_t nFrameId,
						   int32_t   nDetectedObjects,
	                       DetectedPeopleInfo* oDetectedObjectsInfo)
{

    fprintf(_fpMetaData, "%d, %d,", nFrameId, nDetectedObjects);
	int nCount;
	for(nCount = 0;nCount < nDetectedObjects;nCount++)
	{
		fprintf(_fpMetaData, "%d, %d, %d, %d, %d, %d, %d,",
				oDetectedObjectsInfo->nLabelId,
				oDetectedObjectsInfo->bMotionPresent,
				oDetectedObjectsInfo->bPredicted,
				oDetectedObjectsInfo->TopLeft.x,
				oDetectedObjectsInfo->TopLeft.y,
				oDetectedObjectsInfo->BottomRight.x,
				oDetectedObjectsInfo->BottomRight.y);
		oDetectedObjectsInfo++;
	}
	fprintf(_fpMetaData,"\n");
}

/**
 * Close all file-io objects
 *
 * @return 1 -  Success , 0 Failure (COM port initialization failed)
 */
FILE_IO_STATUS FileIO::Close()
{
    if(_fpIn != NULL)
    {
		if (_file_io_mode == true)
		{
			fclose(_fpIn);
		}
		else
		{
			_camera->stop();
		}
        _fpIn = NULL;
		_camera = NULL;
    }

	if (m_infile.is_open())
	{
		m_infile.close();
	}

    if (_fpAVIWriter != NULL)
    {
        VideoWriter* writer = (VideoWriter*)_fpAVIWriter;
        writer->release();
        _fpAVIWriter = NULL;
    }

	if(_fpMetaData != NULL)
	{
		fclose(_fpMetaData);
		_fpMetaData = NULL;
	}

    return FILE_IO_SUCCESS;
}
