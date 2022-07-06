/******************************************************************************
Copyright(c) 2022 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.  By using this software you agree
to the terms of the associated Analog Devices Software License Agreement.
*******************************************************************************/
/*******************************************************************************
 @file :ParseCmd.h

 @brief :Functions for sending detected head locations via UART

 *********************************************************************************/

#ifndef PARSECMD_H
#define PARSECMD_H 

#include <string>

using namespace std;

#define CMDLINEPARSER_FAIL    -1
#define CMDLINEPARSER_SUCCESS  1

/**
 * Enum storing different display types
 *
 */
typedef enum
{
	E_DISPLAY_IR = 0,
	E_DISPLAY_DEPTH = 1,
	E_DISPLAY_SILHOUETTE = 2
}DisplayMode;

/**
 * Enum storing different capture types
 *
 */
typedef enum
{
	E_CAPTURE_MP = 0,
	E_CAPTURE_QMP = 1
}CaptureMode;
/**
 * This class is used to get commandline parameters and map it to the various options
 *
 */
typedef struct CommandParser{
public:
	string InFile;
    string OutFile;
	string ComPort;
	string szCsvFileName;

    int nNumFrms;
	int nHeightOffset;

    int bEnableMetaDataOutput;
    int bEnableOutputFileWrite;

	DisplayMode nDisplayMode;
	CaptureMode nCaptureMode;

	int nSerialCommFrameRate;

	float nDistanceThresholdTrack;
	int nTimeAfterTracked;
	
	int Parse(int argc, char **argv);
}CommandParser;

#endif
