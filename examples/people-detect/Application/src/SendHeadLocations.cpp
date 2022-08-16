/******************************************************************************
 Copyright(c) 2022 Analog Devices, Inc. All Rights Reserved.

 This software is proprietary to Analog Devices, Inc. and its licensors.  By using this software you agree
 to the terms of the associated Analog Devices Software License Agreement.
 *******************************************************************************/
/*******************************************************************************
 @file :SendHeadLocations.cpp

 @brief :Functions for sending detected head locations via UART

 *********************************************************************************/

#include "SendHeadLocations.h"

//OpenCV is used for display and writing avi files
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;

//Text to display on serial enable/disable button
std::string winName = "Click to enable/disable serial tx";

//Flag on current serial comm status
bool bEnableSerialComm = false;

/**
 * Callback to the button to enable/disable serial write.
 *
 * @param event - Event type, button clicked or not
 * @param x - x position of mouse
 * @param y - y position of mouse
 * @param flags - event flag
 * @param userdata - user data sent over
 * @return None
 */
void callBackFunc(int event, int x, int y, int flags, void* userdata)
{
	//Text position got by visual inspection, change if not correct
	if (event == EVENT_LBUTTONUP)
	{
		//toggle message on button click
		Mat3b Display = Mat3b(80, 350, Vec3b(200, 200, 200));
		if (bEnableSerialComm == false)
		{
			//Change text to disable if currently enabled
			std::string buttonText("Left Click to Disable SerialPort Comm");
			putText(Display, buttonText,
				Point(25, 56),
				FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255));
			bEnableSerialComm = true;
		}
		else
		{
			//Change text to enable if currently enabled
			std::string buttonText("Left Click to Enable SerialPort Comm");
			putText(Display, buttonText,
				Point(25, 56),
				FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0));
			bEnableSerialComm = false;
		}
		imshow(winName, Display);
	}	
}

void ConfigureSerialEnableButton()
{
	Mat3b ButtonDisplay;
	//Image size got by visual inspection, change if needed
	ButtonDisplay = Mat3b(80, 350, Vec3b(200, 200, 200));

	std::string buttonText("Left Click to Enable SerialPort Comm");
	//Text position got by visual inspection, change if not correct
	putText(ButtonDisplay, buttonText,
		Point(25, 56),
		FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0));

	namedWindow(winName, WINDOW_AUTOSIZE);

	// Setup callback function for mouse click	
	setMouseCallback(winName, callBackFunc);

	imshow(winName, ButtonDisplay);
	bEnableSerialComm = false;
}

 /**
  * Constructor for HeadTracker class
  *
  */
HeadTracker::HeadTracker()
{
	nTrackedObjects = 0;
	nTxString[0] = '\0';
	hPort = NULL;
	nHeightDiff = 0;
	for (int i = 0; i < MAX_DETECTIONS_TO_SEND; i++)
	{
		oObjects[i].nCenX = 0;
		oObjects[i].nCenY = 0;
		oObjects[i].nCenZ = 0;
		oObjects[i].nIndex = 0;
	}	
}

/**
 * Destructor for HeadTracker class
 */
HeadTracker::~HeadTracker()
{
	nTrackedObjects = 0;
	nTxString[0] = '\0';
	hPort = NULL;
	nHeightDiff = 0;
}

/**
 * Initializes the COM port based on name given as input
 *
 * @param ComPortName - COM port to connect to, eg COM7
 * @param nHeightOffset - Difference in height from sensor to audio beam forming setup
 * @return 1 -  Success , 0 Failure (COM port initialization failed)
 */
bool HeadTracker::InitializeCOMPort(const char* ComPortName,int nHeightOffset)
{
	nHeightDiff = nHeightOffset;

	//Open COM port named ComPortName only in write mode
    DCB dcb;
    hPort = CreateFileA(ComPortName,
                        GENERIC_WRITE,0,NULL,
                        OPEN_EXISTING,0,NULL);

	//If GetCommState is false, this has failed
    if (!GetCommState(hPort,&dcb))
    {
        return false;
    }

	//Default parameters used for communication
    dcb.BaudRate = CBR_9600; //9600 Baud
    dcb.ByteSize = 8; //8 data bits
    dcb.Parity = NOPARITY; //no parity
    dcb.StopBits = ONESTOPBIT; //1 stop

	//If setting parameters fails, return false
    if (!SetCommState(hPort,&dcb))
    {
        return false;
    }

	//Send a char over COM port to indicate success
	//DWORD byteswritten;
	//bool retVal = WriteFile(hPort, "0", 1, &byteswritten, NULL);

	//Enable serial port button
	ConfigureSerialEnableButton();

    return true;
} 

/**
 * Close the COM port after communication is done
 *
 */
void HeadTracker::CloseCOMPort()
{
	//If port was opened, try to close this
	if (hPort != NULL)
	{
		CloseHandle(hPort); //close the handle
	}
}

/**
 * Populate tracked objects from detected head locations
 *
 * @param oHeadObj - Object of DetectedPeopleInfo storing head detections
 * @param oPeopleObj - Object of DetectedPeopleInfo storing people detections 
 * @param nCount - Number of detections
 * @return None
 */
void HeadTracker::PopulateTrackedObjects(DetectedPeopleInfo* oHeadObj,
										 DetectedPeopleInfo* oPeopleObj,
										 int nCount)
{
	//Only send MAX_DETECTIONS_TO_SEND detections
	if (nCount > MAX_DETECTIONS_TO_SEND)
	{
		nCount = MAX_DETECTIONS_TO_SEND;
	}

	nTrackedObjects = nCount;	

	DetectedPeopleInfo* oHeadLocation = oHeadObj;
	//Holds the tracked of each detected object so we can sort and allocate Ids
	//from 1-5
	std::vector<int> nTrackedIDOfObject;

	//Populate oObjects object from oHeadObj
	int nIndex;
	for (nIndex = 0; nIndex < nCount; nIndex++)
	{		
		oObjects[nIndex].nIndex = nIndex;
		nTrackedIDOfObject.push_back(oObjects[nIndex].nIndex);
		oObjects[nIndex].nCenX = oHeadLocation->nLocation.x / 10;//convert mm to cm
		oObjects[nIndex].nCenY = oHeadLocation->nLocation.y / 10;//convert mm to cm
		oObjects[nIndex].nCenY += nHeightDiff;
		oObjects[nIndex].nCenZ = oHeadLocation->nLocation.z / 10;//convert mm to cm			
		oHeadLocation++;
	}

	//DESIGN NOTE - Melody unit can do beamforming for 5 people
	// The selection of people is done using their IDs
	// Internally we allocate IDs based on the tracked id
	// This helps same person get same id and same color
	// However if we internally restrict this number to 5 there might
	// not be enough since we retain it even if person is not in frame
	// We can use mod 5 to change tracked ids to 0-4 but in that case
	// 2 tracked objects can be mapped to same id
	//we sort the tracked ids and then allocate based on tracked id
	//smallest track id -> 1 and so on
	//if there are more than 5 people furthest person may get 1 for whom
	//we cannot beam form. 
	//so if we sort the tracked ids based on distance, it is guaranteed nearest 5 people
	//can be projected to, but there is a chance that as people move, ids will change

	//sort based on tracked id and allocate ids from 1-5
	sort(nTrackedIDOfObject.begin(), nTrackedIDOfObject.end());
	oHeadLocation = oHeadObj;
	for (int i = 0; i < nCount; i++)
	{
		for (int j = 0; j < nTrackedIDOfObject.size(); j++)
		{
			if (nTrackedIDOfObject[j] == oObjects[i].nIndex)
			{
				//update 
				oObjects[i].nIndex = j + 1;

				//make sure display has same index as what we are sending
				//people and heads are displayed in same colour
				oHeadLocation->nLabelId = oObjects[i].nIndex;
				int nPeopleIndex = oHeadLocation->nLinkedIndex;
				oPeopleObj[nPeopleIndex].nLabelId = oObjects[i].nIndex;
				oHeadLocation++;
				break;
			}
		}
	}

	return;
}

/**
 * Send tracked objects over COM port
 *
 */
void HeadTracker::SendTrackedObjects()
{
	if (bEnableSerialComm == true)
	{
		int nIndex;
		//Populate count of tracked objects
		sprintf(nTxString, "%03d", nTrackedObjects);
		for (nIndex = 0; nIndex < nTrackedObjects; nIndex++)
		{
			//Add object index
			sprintf(nTxString + nIndex * 16 + 3, "%04d", oObjects[nIndex].nIndex);

			//First bit denotes sign, 1 indicates -ve, 0 indicates +ve direction
			if (oObjects[nIndex].nCenX < 0)
			{
				nTxString[nIndex * 16 + 7] = '1';
				//Add x location
				sprintf(nTxString + nIndex * 16 + 8, "%03d", -oObjects[nIndex].nCenX);
			}
			else
			{
				nTxString[nIndex * 16 + 7] = '0';
				//Add x location
				sprintf(nTxString + nIndex * 16 + 8, "%03d", oObjects[nIndex].nCenX);
			}

			//First bit denotes sign, 1 indicates -ve, 0 indicates +ve direction
			if (oObjects[nIndex].nCenY < 0)
			{
				nTxString[nIndex * 16 + 11] = '1';
				//Add y location
				sprintf(nTxString + nIndex * 16 + 12, "%03d", -oObjects[nIndex].nCenY);
			}
			else
			{
				nTxString[nIndex * 16 + 11] = '0';
				//Add y location
				sprintf(nTxString + nIndex * 16 + 12, "%03d", oObjects[nIndex].nCenY);
			}

			//First bit denotes sign, 1 indicates -ve, 0 indicates +ve direction
			if (oObjects[nIndex].nCenZ < 0)
			{
				nTxString[nIndex * 16 + 15] = '1';
				//Add z location
				sprintf(nTxString + nIndex * 16 + 16, "%03d", -oObjects[nIndex].nCenZ);
			}
			else
			{
				nTxString[nIndex * 16 + 15] = '0';
				//Add z location
				sprintf(nTxString + nIndex * 16 + 16, "%03d", oObjects[nIndex].nCenZ);
			}
		}

		//set remaining bits in the output string to 0
		memset(nTxString + (nTrackedObjects * 16 + 3), '0', TX_BUFF_SIZE - 16 * nTrackedObjects - 3);
		nTxString[TX_BUFF_SIZE - 1] = '\0';

		//Send this via UART port if COM port has been opened
		DWORD byteswritten;
		if (hPort != NULL)
		{
			bool retVal = WriteFile(hPort, nTxString, TX_BUFF_SIZE, &byteswritten, NULL);
			//retVal = WriteFile(hPort, "\r\n", 2, &byteswritten, NULL);
		}
	}
	return;
}



