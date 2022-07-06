/******************************************************************************
Copyright(c) 2022 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.  By using this software you agree
to the terms of the associated Analog Devices Software License Agreement.
*******************************************************************************/
/*******************************************************************************
 @file :SendHeadLocations.h

 @brief :Functions for sending detected head locations via UART

 *********************************************************************************/

#ifndef SENDHEADLOCATIONS_H
#define SENDHEADLOCATIONS_H 

//for sending over COM port
#include <windows.h>
#include <string>
#include <iostream>
#include "PeopleDetector.h"

//Maximum number of detections to send via UART
#define MAX_DETECTIONS_TO_SEND    5

//Transmission string size
#define TX_BUFF_SIZE            128

/*! \class TrackedHeadObjects
	\brief A class used for holding location of objects to be sent via UART.

	This structure holds the 3D location of detected objects to be sent via UART.
*/
class TrackedHeadObjects
{
public:
	int nIndex;
	int nCenX;
	int nCenY;
	int nCenZ;
};

/*! \class TrackedHeadObjects
	\brief A class used for holding location of objects to be sent via UART.

	This structure holds the 3D location of detected objects to be sent via UART.
*/
class HeadTracker
{
private:
	char nTxString[TX_BUFF_SIZE];
	int nTrackedObjects;
	int nHeightDiff;
	HANDLE hPort;
	TrackedHeadObjects oObjects[MAX_DETECTIONS_TO_SEND];
public:
	void PopulateTrackedObjects(DetectedPeopleInfo* oHeadObj, 
		DetectedPeopleInfo* oPeopleObj, int nCount);
	void SendTrackedObjects();
	bool InitializeCOMPort(const char* ComPortName, int nHeightOffset);
	void CloseCOMPort();
	HeadTracker();
	~HeadTracker();
};

#endif
