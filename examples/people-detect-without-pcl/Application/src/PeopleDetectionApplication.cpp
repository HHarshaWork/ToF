#include <iostream>
#include <chrono>
#include "SendHeadLocations.h"
#include "PeopleDetector.h"
#include "ParseCmd.h"
#include "FileIO.h"

#define QMP_MODE 7

//OpenCV is used for display and writing avi files
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

#include <iostream>
#include <fstream>
#include <ios>

using namespace std::chrono;

/* Image Buffers */
#define MAX_NUM_COLORS 30

/**
 * Draw Silhouette on Mat image.
 *
 * @param OutputRGBImage - Output cv::mat image in rgb format
 * @param silhouette - Silhouette mask
 * @param det - DetectedPeopleInfo object holding details of current object
 * @return None
 */
void DrawSilhouette(Mat OutputRGBImage, 
					uint8_t* silhouette, 
					DetectedPeopleInfo* det)
{
	unsigned char color[MAX_NUM_COLORS][3] = {
		{ 255, 255, 0 },{ 0, 255, 255 },{ 255, 0, 255 },
		{ 128, 128, 255 },{ 128, 128, 0 },{ 128, 0, 128 },
		{ 0, 128, 128 },{ 0, 0, 128 },{ 0, 128, 0 },
		{ 65, 128, 65 },{ 65, 65, 128 },{ 128, 65, 65 },
		{ 255, 30, 30 },{ 30, 255, 30 },{ 30, 30, 255 },
		{ 175, 255, 175 },{ 175, 175, 255 },{ 255, 175, 175 },
		{ 68, 200, 93 },{ 200, 68, 93 },{ 93, 68, 200 },
		{ 187, 214, 54 },{ 54, 214, 187 },{ 214, 187, 54 },
		{ 78, 60, 208 },{ 208, 78, 60 },{ 60, 208, 78 },
		{ 255, 0, 0 },{ 0, 255, 0 },{ 0, 0, 255 },
	};

	int width = OutputRGBImage.cols;

	//Draw only in silhouette
	for (int nRow = det->TopLeft.y; nRow < det->BottomRight.y; nRow++)
	{
		uint8_t* DrawPtr = (uint8_t*)OutputRGBImage.data +
			(nRow * width * 3 + det->TopLeft.x * 3);//x,y,z

		uint8_t* DrawSil = silhouette + (nRow * width + det->TopLeft.x);

		for (int nCol = det->TopLeft.x; nCol < det->BottomRight.x; nCol++) {
			if (*DrawSil)
			{
				*(DrawPtr + 0) = int((color[det->nLabelId][0] * 5) / 10);//b
				*(DrawPtr + 1) = int((color[det->nLabelId][1] * 5) / 10);//g
				*(DrawPtr + 2) = int((color[det->nLabelId][2] * 5) / 10);//r
			}
			DrawPtr += 3;
			DrawSil += 1;
		}
	}
}

/**
 * Draw Detected objects on Mat image.
 *
 * @param pDispImage - Output cv::mat image in rgb format
 * @param nObjectCount - Number of objects
 * @param poDetectedObjects - DetectedPeopleInfo objects holding details of current object
 * @param nClassID - ID of object to determine colour to use 
 * @return None
 */
void DrawDetectedObjects(Mat &pDispImage,
						 int32_t nObjectCount,
						 DetectedPeopleInfo *poDetectedObjects, 
						 int nClassID)
{
	int32_t nNumTrackedObjects = nObjectCount;
	char textBuf[100];

	int nWidth = pDispImage.cols;
	unsigned char color[MAX_NUM_COLORS][3] = {

		{ 255, 255, 0 },{ 0, 255, 255 },{ 255, 0, 255 },
		{ 128, 128, 255 },{ 128, 128, 0 },{ 128, 0, 128 },
		{ 0, 128, 128 },{ 0, 0, 128 },{ 0, 128, 0 },
		{ 65, 128, 65 },{ 65, 65, 128 },{ 128, 65, 65 },
		{ 255, 30, 30 },{ 30, 255, 30 },{ 30, 30, 255 },
		{ 175, 255, 175 },{ 175, 175, 255 },{ 255, 175, 175 },
		{ 68, 200, 93 },{ 200, 68, 93 },{ 93, 68, 200 },
		{ 187, 214, 54 },{ 54, 214, 187 },{ 214, 187, 54 },
		{ 78, 60, 208 },{ 208, 78, 60 },{ 60, 208, 78 },
		{ 255, 0, 0 },{ 0, 255, 0 },{ 0, 0, 255 }
	};

	int nRemoveMotionCount = nObjectCount;
	for (int i = 0; i < nNumTrackedObjects; i++)
	{
		/*draw bounding box on object */
		int nId = poDetectedObjects[i].nLabelId % MAX_NUM_COLORS;

		if (nClassID == HEAD_CLASS)
		{
			int nWidth = poDetectedObjects[i].BottomRight.x - poDetectedObjects[i].TopLeft.x;

			sprintf(textBuf, "ID-%1d", nId);
			cv::putText(pDispImage, textBuf,
				Point2i(poDetectedObjects[i].TopLeft.x - nWidth/2,
					max(0,poDetectedObjects[i].TopLeft.y - 20)), 
				FONT_HERSHEY_PLAIN, 5.0f,
				cv::Scalar(255, 255, 255, 0), 
				5, 8, false);
		}
		/* Draw rectangle if non-predicted object, else draw circle */
		if (poDetectedObjects[i].bPredicted == false)
		{
			cv::rectangle(pDispImage,
				cv::Point((int32_t)(poDetectedObjects[i].TopLeft.x), (int32_t)(poDetectedObjects[i].TopLeft.y)),
				cv::Point((int32_t)(poDetectedObjects[i].BottomRight.x), (int32_t)(poDetectedObjects[i].BottomRight.y)),
				cv::Scalar(color[nId][0], color[nId][1], color[nId][2], 0), 2);
		}
		else
		{
			int nRadius = min(poDetectedObjects[i].BottomRight.x - poDetectedObjects[i].TopLeft.x,
				poDetectedObjects[i].BottomRight.y - poDetectedObjects[i].TopLeft.y);
			cv::circle(pDispImage,
				cv::Point((int32_t)(poDetectedObjects[i].TopLeft.x + poDetectedObjects[i].BottomRight.x) / 2,
					(int32_t)(poDetectedObjects[i].TopLeft.y + poDetectedObjects[i].BottomRight.y) / 2), nRadius,
				cv::Scalar(color[nId][0], color[nId][1], color[nId][2], 0), 2);
		}

		//If object is in motion,draw a dot in centre to indicate
		//also do not count
		if (poDetectedObjects[i].bMotionPresent == true)
		{
			nRemoveMotionCount -= 1;
			cv::circle(pDispImage,
				cv::Point((int32_t)(poDetectedObjects[i].TopLeft.x + poDetectedObjects[i].BottomRight.x) / 2,
					(int32_t)(poDetectedObjects[i].TopLeft.y + poDetectedObjects[i].BottomRight.y) / 2), 10,
				cv::Scalar(color[nId][0], color[nId][1], color[nId][2], 0), -1);
		}

		//draw 2d tracking trajectory
		if (nClassID == PERSON_CLASS)
		{
			for (int j = 0; j < poDetectedObjects[i].nTrajectoryPoints; j++)
			{
				cv::circle(pDispImage,
					cv::Point((int32_t)(poDetectedObjects[i].o2DTrajectory[j].x),
						(int32_t)(poDetectedObjects[i].o2DTrajectory[j].y)), 4,
					cv::Scalar(color[nId][0], color[nId][1], color[nId][2], 0), -1);
			}
		}
	}	

	//Count of heads to be displayed in image
	if (nClassID == HEAD_CLASS)
	{
		sprintf(textBuf, "COUNT : %3d", nRemoveMotionCount);
		cv::putText(pDispImage, textBuf,
			Point(nWidth / 2, 20), FONT_HERSHEY_PLAIN, 1.5f,
			Scalar(0, 0, 255, 255), 2, 8, false);
	}
}

// Convert hsv floats ([0-1],[0-1],[0-1]) to rgb floats ([0-1],[0-1],[0-1]), from Foley & van Dam p593
// also http://en.wikipedia.org/wiki/HSL_and_HSV
void ColorConvertHSVtoRGB(float h, float s, float v, float& out_r, 
	float& out_g, float& out_b)
{
	if (s == 0.0f)
	{
		// gray
		out_r = out_g = out_b = v;
		return;
	}

	h = fmodf(h, 1.0f) / (60.0f / 360.0f);
	int   i = (int)h;
	float f = h - (float)i;
	float p = v * (1.0f - s);
	float q = v * (1.0f - s * f);
	float t = v * (1.0f - s * (1.0f - f));

	switch (i)
	{
		case 0: out_r = v; out_g = t; out_b = p; break;
		case 1: out_r = q; out_g = v; out_b = p; break;
		case 2: out_r = p; out_g = v; out_b = t; break;
		case 3: out_r = p; out_g = q; out_b = v; break;
		case 4: out_r = t; out_g = p; out_b = v; break;
		case 5: default: out_r = v; out_g = p; out_b = q; break;
	}
}

/**
 * Create a heat map display
 *
 * @param pSource - Source image in uint16 format 1 channel
 * @param pDest - Output image in RGB 8-bit format
 * @param nWidth - width of input image
 * @param nHeight - height of input image
 * @return None
 */
void createHeatMap(uint16_t* pSource, 
				   uint8_t* pDest, 
				   int32_t nWidth, 
				   int32_t nHeight)
{
	int nMaxInDepth = 5000;
	int nMinDepth = 0;
	int nMaxOutDepth = 255;

	for (int i = 0; i < nWidth * nHeight; i++)
	{
		int nCurrentValue = *pSource++;
		nCurrentValue = std::min(nCurrentValue, nMaxInDepth);
		nCurrentValue = std::max(nCurrentValue, nMinDepth);
		// The 'hue' coordinate in HSV is a polar coordinate, so it 'wraps'.
		// Purple starts after blue and is close enough to red to be a bit unclear,
		// so we want to go from blue to red.  Purple starts around .6666667,
		// so we want to normalize to [0, .6666667].
		//
		float hue = (nCurrentValue - nMinDepth) / static_cast<float>(nMaxInDepth - nMinDepth);
		constexpr float range = 2.f / 3.f;
		hue *= range;
		// We want blue to be close and red to be far, so we need to reflect the
		// hue across the middle of the range.
		//
		hue = range - hue;

		float fRed = 0.f;
		float fGreen = 0.f;
		float fBlue = 0.f;
		ColorConvertHSVtoRGB(hue, 1.f, 1.f, fRed, fGreen, fBlue);

		//opencv needs it as bgr
		*pDest++ = static_cast<uint8_t>(fBlue * nMaxOutDepth);
		*pDest++ = static_cast<uint8_t>(fGreen * nMaxOutDepth);
		*pDest++ = static_cast<uint8_t>(fRed * nMaxOutDepth);
	}
}

/**
 * Main function
 *
 */
int main(int argc, char** argv)
{
	try {
		uint32_t nFrameCount;
		int nWidth, nHeight;

		std::cout << "People Detection Application Started...." << std::endl;

		//Output display window
		string szDisplayWindowName("People Detector Output");
		namedWindow(szDisplayWindowName, WINDOW_NORMAL);

		//parse and read all arguments
		CommandParser oCommandParser;
	
		/* Parse and read cmd line parameters from example.cmd*/
		oCommandParser.Parse(argc, argv);

		/* If capture mode is set to quarter megapixel, adjust width and height accordingly*/
		if (oCommandParser.nCaptureMode == CaptureMode::E_CAPTURE_QMP)
		{
			nWidth = 512;
			nHeight = 512;
		}
		else
		{
			nWidth = 1024;
			nHeight = 1024;
		}

		//Initialize CV mat RGB image for output display
		Mat ganRGBImage(nWidth, nHeight, CV_8UC3);

		//Initialize file io object
		FileIO oFileIO(nWidth, nHeight);

		/* Open file io based on user inputs*/
		if (oCommandParser.bEnableOutputFileWrite == 1)
		{
			if (!oFileIO.Init((char *)oCommandParser.InFile.c_str(), 
				(char *)oCommandParser.szCsvFileName.c_str(), (char *)oCommandParser.OutFile.c_str(),
				(int)oCommandParser.nCaptureMode))
			{
				cout << "Error: File io init failed" << endl;
				return -1;
			}
		}
		else
		{
			if (oCommandParser.bEnableMetaDataOutput == 1)
			{
				if (!oFileIO.Init((char *)oCommandParser.InFile.c_str(), 
					(char *)oCommandParser.szCsvFileName.c_str(), (int)oCommandParser.nCaptureMode))
				{
					cout << "Error: File io init failed" << endl;
					return -1;
				}
			}
			else
			{
				if (!oFileIO.Init((char *)oCommandParser.InFile.c_str(), (int)oCommandParser.nCaptureMode))
				{
					cout << "Error: File io init failed" << endl;
					return -1;
				}
			}
		}

		/* Set input frame size*/
		oFileIO.SetInputFrameSize(nWidth, nHeight);

		/* Initialize People Detector object*/
		PeopleDetector oPeopleDetector(nWidth, nHeight);

		//Enable tracking mode in people detector
		oPeopleDetector.SetPeopleTracking(true);

		//Configure the people detector module
		bool bConfigured = oPeopleDetector.ConfigModule();
		if (bConfigured == false)
		{
			//tracker has failed so do not continue;
			cout << "Tracker has not been initialized correctly so stopping here" << endl;
			return -1;
		}

		//update config from commandline input fron user
		ConfigParamsPeopleDetector oPeopleParams;
		oPeopleParams.DistanceThresholdForStaticObjects = oCommandParser.nDistanceThresholdTrack;
		oPeopleParams.timeToStartCounting = oCommandParser.nTimeAfterTracked;
		oPeopleDetector.SetParams(&oPeopleParams);

		//Initialize input buffer used for reading image
		//image format is depth image(1),IR image(1),point cloud image(3)
		int32_t nSize = nWidth * nHeight * sizeof(uint16_t) * 5;
		uint8_t* pImage = new uint8_t[nSize];
		if (pImage == NULL)
		{
			cout << "Error: Memory for Image could not be allocated" << endl;
			return -1;
		}

		//Initialize object to hold final tracked objects
		HeadTracker oTrackedHeadObjects;

		//Initialize COM port to send the tracked objects over UART
		oTrackedHeadObjects.InitializeCOMPort(oCommandParser.ComPort.c_str(), oCommandParser.nHeightOffset);

		//Main processing loop
		while (true)
		{
			//If frame counter is beyond num of frame so run, exit loop
			nFrameCount = oFileIO.GetFrameCounter();
			if (nFrameCount >= (uint32_t)oCommandParser.nNumFrms)
			{
				cout << "All frames processed\n" << endl;
				break;
			}		

			/* Read video frame from file */
			if (oFileIO.Read(pImage) == FILE_IO_FAILURE)
			{
				//If file is done, exit
				cout << "File io read failed\n" << endl;
				break;
			}

			//Timer to measure processing time
			auto start = high_resolution_clock::now();

			//Inject frame to detector
			oPeopleDetector.InjectFrame(pImage);

			//Run detection on input image
			oPeopleDetector.Run();

			auto stop = high_resolution_clock::now();
			auto duration = duration_cast<milliseconds>(stop - start);
				
			int32_t numHeadDet, numPeopleDet;
			DetectedPeopleInfo oHead[MAX_DETECTED_OBJECTS];
			DetectedPeopleInfo oPeople[MAX_DETECTED_OBJECTS];

	#if 0
			//Get all detections of people and heads
			DetectedPeopleInfo *oPeople;
			oPeople = oPeopleDetector.DetectStaticPeople(numPeopleDet);
			DetectedPeopleInfo* oHead;
			oHead = oPeopleDetector.DetectHead(numHeadDet);
	#else 
			//Get location of people and their heads if static
			oPeopleDetector.GetLocationHead(numHeadDet, numPeopleDet, oPeople, oHead);
	#endif

			int32_t nBufSize = ganRGBImage.rows * ganRGBImage.cols;
			if (oCommandParser.nDisplayMode == E_DISPLAY_IR)
			{
				//Get processed IR image for display
				uint8_t* pIRImg = oPeopleDetector.GetProcessedIRImage();
				memcpy(ganRGBImage.data, pIRImg, nBufSize * 3);
			}
			else if(oCommandParser.nDisplayMode == E_DISPLAY_DEPTH)
			{
				//Get processed depth image for display
				uint16_t* pDepthImg = oPeopleDetector.GetProcessedDepthImage();

				//convert to heat map representation
				createHeatMap(pDepthImg, ganRGBImage.data,
					nWidth, nHeight);
			}
			else
			{
				//Get silhouette image for display
				uint8_t* mask = oPeopleDetector.GetSilhouette();

				memset(ganRGBImage.data, 0, nBufSize * 3);
				//Draw silhoutte on output image
				for (int nPeople = 0; nPeople < numPeopleDet; nPeople++)
				{
					DrawSilhouette(ganRGBImage, mask, &oPeople[nPeople]);
				}
			}

			//populate heads of people tracked and tx and send over UART
			oTrackedHeadObjects.PopulateTrackedObjects(oHead, oPeople, numHeadDet);
			if (nFrameCount % oCommandParser.nSerialCommFrameRate == 0)
			{
				oTrackedHeadObjects.SendTrackedObjects();
			}
			/* Draw bounding box for people and heads */
			DrawDetectedObjects(ganRGBImage, numPeopleDet, oPeople, PERSON_CLASS);
			DrawDetectedObjects(ganRGBImage, numHeadDet, oHead, HEAD_CLASS);

			//draw detections and write to csv file
			/* Write to file, used only in file-io*/
			if (oCommandParser.bEnableMetaDataOutput == 1)
			{
				oFileIO.WriteMetaData(nFrameCount,
					numHeadDet, oHead);
			}

			/* Show output with graphics*/
			imshow(szDisplayWindowName, ganRGBImage);
			int32_t res = waitKey(1);
			if (res == 27)
			{
				//escape key is pressed, break
				break;
			}
		
			/* Write output to avi file */
			oFileIO.Write(ganRGBImage.data, nWidth*nHeight);

			printf("Frame No - %d, Count - %d, Time Taken - %lld ms\n", 
				nFrameCount, numHeadDet, duration.count());
		}

		//close com port connection
		oTrackedHeadObjects.CloseCOMPort();

		//close detector object
		oPeopleDetector.CloseModule();

		//close fileio objects
		oFileIO.Close();

		return 0;
	} catch(...) {
		cout << "Exception Caught \n";
	}
}


