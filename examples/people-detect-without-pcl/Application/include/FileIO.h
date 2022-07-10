/******************************************************************************
 Copyright(c) 2018 Analog Devices, Inc. All Rights Reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.  By using this software you agree
 to the terms of the associated Analog Devices Software License Agreement.
 *******************************************************************************/
/*******************************************************************************
 @file :FileIO.h

 @brief :Functions for reading and writing files

 *********************************************************************************/

#ifndef __FILE_IO_H__
#define __FILE_IO_H__

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include "PeopleDetector.h"

//Used to interface tof camera for realtime mode
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/system.h>
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <glog/logging.h>

using namespace std;
using namespace aditof;

#define FILE_IO_FAILURE			0       // Return value for failure
#define FILE_IO_SUCCESS 		1       // Return value for success

/**
 * Return status of file io operations
 *
 */
typedef int32_t FILE_IO_STATUS;

/**
 * Class used for file input/output operations
 *
 */
class FileIO
{
private:
	FILE *_fpIn;
	FILE *_fpMetaData;
	ifstream m_infile;
	void *_fpAVIWriter;

	int32_t _nInputFrameSize;
	int32_t _nFrameCounter;
	uint32_t _nOutWidth;
	uint32_t _nOutHeight;

	std::shared_ptr<aditof::Camera> _camera;//used to get frames from ToF camera

	bool     _file_io_mode;

	FILE_IO_STATUS OpenInputFile(char *pInFileName, int nMode);
	FILE_IO_STATUS OpenOutputFile(char *pOutFileName);
	FILE_IO_STATUS OpenMetadataFile(char *pMetaDataFileName);

public:
	FileIO();

	FileIO(int width, int height);

	FILE_IO_STATUS Init(char *pInFileName, int nMode);

	FILE_IO_STATUS Init(char *pInFileName, char *pMetaDataFileName, int nMode);

	FILE_IO_STATUS Init(char *pInFileName, char *pMetaDataFileName, char *pOutFileName, int nMode);

	int32_t GetFrameCounter();

	void SetInputFrameSize(uint32_t nWidth, uint32_t nHeight);

	FILE_IO_STATUS Read(uint8_t *pBuf);

	FILE_IO_STATUS Write(uint8_t *pBuf, uint32_t nNumBytesToWrite);

	void WriteMetaData(int32_t nFrameId,			           
					   int32_t nNumDetectedObjects,
		               DetectedPeopleInfo* oDetectedObjectsInfo);

	FILE_IO_STATUS Close();

};

#endif /* __FILE_IO_H__ */
