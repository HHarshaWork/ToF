#include <iostream>
#include <fstream>
#include "PeopleDetector.h"
#include "object_track.h"
#include "../include/image_filtering.h"

using namespace std;
using namespace cv;

#define BLOCK_SIZE 20
#define DEPTH_THRESHOLD 500
#define DEPTH_CONSTANT 200 // DEPTH * (WIDTH OF HEAD) = Constant
#define MAX_DEPTH 7000 // max depth is taken as 7m
#define PEOPLE_DETECTION_THRESHOLD 0.3f
#define HEAD_DETECTION_THRESHOLD   0.5f

//utility functions for silhouette
void ThresholdDepthImageOnROI(uint16_t* pDepthImage, uint16_t* pSilhouetteImage, 
	int16_t nRows, int16_t nCols, DetectedObjectInfo* pBBox, int index, int16_t nDepthThreshold, int16_t nCenDepth);

cv::Point GetCentroidOfMaxObjectOnROI( cv::Mat thr, 
	int16_t nRows, int16_t nCols, DetectedObjectInfo* pBBox);

cv::Point GetMaximumHeadPointOnSilloheutteOnROI(cv::Mat thr, DetectedObjectInfo* pBBox, 
	int head_width, Mat drawing);

void regionGrow(uint16_t* img, uint16_t* outimg, int32_t depthCols, 
	DetectedObjectInfo* roiparams, int32_t seed_x, int32_t seed_y, int32_t threshold, 
	int nLabel, int16_t referenceVal);

#define FRAMERATE 1
#define HEAD_TRACKER_TRACK_MULT_FACTOR 5
#define PEOPLE_TRACKER_TRACK_MULT_FACTOR 1

void PeopleDetector::SetDefaultParams()
{
	params.DistanceThresholdForStaticObjects = 1 * 30.48f; /* in cm */
	params.timeToStartCounting = 5; /* should be 30 secs */
}

void PeopleDetector::SetParams(ConfigParamsPeopleDetector* prms)
{
	params.DistanceThresholdForStaticObjects = prms->DistanceThresholdForStaticObjects;
	params.timeToStartCounting = prms->timeToStartCounting;
}

float CheckDistanceTravelled(int16_t X1,
	int16_t Y1,
	int16_t Z1,
	int16_t X2,
	int16_t Y2,
	int16_t Z2)
{
	uint32_t nDist = (X1 - X2) * (X1 - X2);
	nDist += (Y1 - Y2) * (Y1 - Y2);
	nDist += (Z1 - Z2) * (Z1 - Z2);
	return(sqrt((float)nDist));
}

uint16_t ComputeAverageDepth(int16_t* pPointCloudImage, uint16_t nWidth, uint16_t nHeight,
	int16_t* nXCentroid, int16_t* nYCentroid)
{
	uint16_t nDepth = 0;

	int nWindowSize = POINTCLOUD_WINDOW_SIZE;

	int nXMin = std::max(0, *nXCentroid - nWindowSize);
	int nYMin = std::max(0, *nYCentroid - nWindowSize);
	int nXMax = std::min(nWidth - 1, *nXCentroid + nWindowSize);
	int nYMax = std::min(nHeight - 1, *nYCentroid + nWindowSize);

	int nSumX = 0;
	int nSumY = 0;
	int nSumZ = 0;
	int nValidPixels = 0;
	for (int j = nYMin; j <= nYMax; j++)	
	{
		int16_t* pData = pPointCloudImage + j * nWidth * 3 + nXMin * 3;
		for (int i = nXMin; i <= nXMax; i++)
		{
			if (*pData != 0)
			{
				nValidPixels += 1;
				nSumX += *pData;
				nSumY += *(pData + 1);
				nSumZ += *(pData + 2);
			}
			pData += 3;
		}
	}

	if (nValidPixels != 0)
	{
		nDepth = nSumZ / nValidPixels;
		*nXCentroid = nSumX / nValidPixels;
		*nYCentroid = nSumY / nValidPixels;
	}
	return nDepth;
}

bool PeopleDetector::ConfigModule()
{
	if (PeopleTracker->IsInitialized() == false)
	{
		return false;
	}
	if (HeadTracker->IsInitialized() == false)
	{
		return false;
	}
	PeopleTracker->SetFramerate(FRAMERATE);
	HeadTracker->SetFramerate(FRAMERATE);
	PeopleTracker->SetThresholdMultFactor(PEOPLE_TRACKER_TRACK_MULT_FACTOR);
	HeadTracker->SetThresholdMultFactor(HEAD_TRACKER_TRACK_MULT_FACTOR);
	PeopleTracker->Enable3DTracking(true);
	HeadTracker->Enable3DTracking(false);
	PeopleTracker->SetDepthImageDim(inpWidth, inpHeight);
	HeadTracker->SetDepthImageDim(inpWidth, inpHeight);
	ObjectDetectorParams oParams;
	oParams.fHeadConfThreshold = 0.5f;
	oParams.fPeopleConfThreshold = 0.3f;
	oParams.fNMSThreshold = 0.4f;
	oParams.nInpWidth = inpWidth;
	oParams.nInpHeight = inpHeight;
	pobjDet->ConfigModule(&oParams);
	bool bIsInitialized = true;
	return true;
}

void PeopleDetector::InjectFrame(uint8_t* pImage)
{
	int32_t nBufSzForImage = inpWidth * inpHeight * elementSize;

	memcpy(pDepthImage, pImage, nBufSzForImage);
	memcpy(pPointCloudImage, pImage + 2 * nBufSzForImage, 3 * nBufSzForImage);

	//process IR image with log correction
	cv::Mat Temp16BitIRImage(inpWidth, inpHeight, CV_16UC1);
	memcpy(Temp16BitIRImage.data, pImage + nBufSzForImage, inpWidth * inpHeight * sizeof(uint16_t));
	memset(pSilhouette, 0, inpWidth * inpHeight * 1);

	cv::Mat NormalizedImage;
	cv::Mat Temp8BitIRImage;
	cv::Mat IRImage;

	GammaCorrect((uint16_t*)Temp16BitIRImage.data, Temp16BitIRImage.rows * Temp16BitIRImage.cols);

	//convert to 8 bit image
	Temp16BitIRImage.convertTo(Temp8BitIRImage, CV_8UC1, 1, 0);//convert to 8 bit

	cvtColor(Temp8BitIRImage, IRImage, cv::COLOR_GRAY2RGB);//convert to rgb 8 bit
	memcpy(pIRImage, IRImage.data, inpWidth * inpHeight * 3);
	pobjDet->SetIRImage(pIRImage);	

	//Filter the depth image
	FilterDepth();
}

void PeopleDetector::FilterDepth()
{
	int32_t nBufSzForImage = inpWidth * inpHeight * elementSize;

	//process depth image
	memset(pFilteredDepthImage, 0, nBufSzForImage);
	MedianFilter(pDepthImage, pFilteredDepthImage, inpWidth, inpHeight,
		KERNEL_SIZE_3x3);
	uint16_t mask[9];
	for (int i = 0; i < 9; i++)
		mask[i] = 1;
	Dilation(pFilteredDepthImage, pDepthImage, inpWidth, inpHeight, 3, mask);
}

DetectedPeopleInfo* PeopleDetector::DetectPeople(int& numPeople)
{
	//This function gets all the people detected in the room regardless of moving or static

	DetectedObjectInfo objDet[MAX_DETECTED_OBJECTS];
	int32_t numPeopleDetected = pobjDet->GetDetections(objDet, PERSON_CLASS, PEOPLE_DETECTION_THRESHOLD);
	numPeopleDetected = numPeopleDetected < MAX_PEOPLE_DETECTION ? numPeopleDetected : MAX_PEOPLE_DETECTION;
	
	ObjFeat* PeopleFeatures = new ObjFeat[numPeopleDetected];
	TrackedObj* trkObjs;
	int nFilteredPeopleCount = 0;
	if (bEnableTrackPeople)
	{
		for (int i = 0; i < numPeopleDetected; i++)
		{
			//aspect ratio changes as per Raka
			int y_start, y_end;
			float ratio1 = (objDet[i].nMaxX - objDet[i].nMinX)
				/ (float)(objDet[i].nMaxY - objDet[i].nMinY);
			float ratio2 = 1 / ratio1;
			if (!bEnableFullBodyTrack && ratio1 <= 0.7)
			{
				y_start = objDet[i].nMinY;
				y_end = ((objDet[i].nMaxY - objDet[i].nMinY) * 6)/10 + y_start;
			}
			else
			{
				y_start = objDet[i].nMinY;
				y_end = objDet[i].nMaxY;
			}

			PeopleFeatures[i].min_x = objDet[i].nMinX;
			PeopleFeatures[i].min_y = y_start;
			PeopleFeatures[i].max_x = objDet[i].nMaxX;
			PeopleFeatures[i].max_y = y_end;

			PeopleFeatures[i].area = (PeopleFeatures[i].max_x - PeopleFeatures[i].min_x) *
				(PeopleFeatures[i].max_y - PeopleFeatures[i].min_y);
			PeopleFeatures[i].centroid_x = (PeopleFeatures[i].max_x + PeopleFeatures[i].min_x) / 2 * PeopleFeatures[i].area;
			PeopleFeatures[i].centroid_y = (PeopleFeatures[i].max_y + PeopleFeatures[i].min_y) / 2 * PeopleFeatures[i].area;
			PeopleFeatures[i].ptr = NULL;
			PeopleFeatures[i].detection_type = DETECTED;
		}

		PeopleTracker->SetDepthImage(pDepthImage, pPointCloudImage);
		PeopleTracker->Process(PeopleFeatures, numPeopleDetected, frame_num);
		PeopleTracker->GetTrackedObjects(&trkObjs, numPplTracked);
				
		for (int i = 0; i < numPplTracked; i++)
		{
			poPeopleDetections[nFilteredPeopleCount].TopLeft.x = trkObjs[i].min_x;
			poPeopleDetections[nFilteredPeopleCount].TopLeft.y = trkObjs[i].min_y;
			poPeopleDetections[nFilteredPeopleCount].BottomRight.x = trkObjs[i].max_x;
			poPeopleDetections[nFilteredPeopleCount].BottomRight.y = trkObjs[i].max_y;
			poPeopleDetections[nFilteredPeopleCount].nBody = UPPER_BODY;
			int16_t nXCentroid = (trkObjs[i].min_x + trkObjs[i].max_x) >> 1;
			int16_t nYCentroid = (trkObjs[i].min_y + trkObjs[i].max_y) >> 1;
			uint16_t nAvgDepth = ComputeAverageDepth(pPointCloudImage, inpWidth, inpHeight, &nXCentroid, &nYCentroid);
			poPeopleDetections[nFilteredPeopleCount].nLocation.x = nXCentroid;
			poPeopleDetections[nFilteredPeopleCount].nLocation.y = nYCentroid;
			poPeopleDetections[nFilteredPeopleCount].nLocation.z = nAvgDepth;
			poPeopleDetections[nFilteredPeopleCount].nLabelId = trkObjs[i].track_label;
			poPeopleDetections[nFilteredPeopleCount].bMotionPresent = false;
			poPeopleDetections[nFilteredPeopleCount].bPredicted = false;
			//Update tracking trajectory
			int nCurrentIndex = trkObjs[i].trkTrajectoryIndx;
			int nTrajecPoints = 0;
			for (int nTrackIndex = 0;
				nTrackIndex < min(trkObjs[i].track_count, NUM_TRACK_POINTS);
				nTrackIndex++)
			{
				poPeopleDetections[nFilteredPeopleCount].o2DTrajectory[nTrackIndex].x
					= trkObjs[i].trkTrajectoryX[nCurrentIndex];
				poPeopleDetections[nFilteredPeopleCount].o2DTrajectory[nTrackIndex].y
					= trkObjs[i].trkTrajectoryY[nCurrentIndex];

				poPeopleDetections[nFilteredPeopleCount].o3DTrajectory[nTrackIndex].x
					= trkObjs[i].PointCloudTrajectoryX[nCurrentIndex];
				poPeopleDetections[nFilteredPeopleCount].o3DTrajectory[nTrackIndex].y
					= trkObjs[i].PointCloudTrajectoryY[nCurrentIndex];
				poPeopleDetections[nFilteredPeopleCount].o3DTrajectory[nTrackIndex].z
					= trkObjs[i].PointCloudTrajectoryZ[nCurrentIndex];

				nCurrentIndex--;
				if (nCurrentIndex < 0)
				{
					nCurrentIndex = NUM_TRACK_POINTS - 1;
				}
				nTrajecPoints++;
			}
			poPeopleDetections[nFilteredPeopleCount].nTrajectoryPoints = nTrajecPoints;
			
			nFilteredPeopleCount += 1;
		}
	}

	if (!bEnableTrackPeople)
	{
		for (int i = 0; i < numPeopleDetected; i++)
		{
			//aspect ratio changes as per Raka
			int y_start, y_end;
			float ratio1 = (objDet[i].nMaxX - objDet[i].nMinX)
				/ (float)(objDet[i].nMaxY - objDet[i].nMinY);
			float ratio2 = 1 / ratio1;
			if (!bEnableFullBodyTrack && ratio1 <= 0.7)
			{
				y_start = objDet[i].nMinY;
				y_end = ((objDet[i].nMaxY - objDet[i].nMinY) * 6)/10 + y_start;
			}
			else
			{
				y_start = objDet[i].nMinY;
				y_end = objDet[i].nMaxY;
			}

			poPeopleDetections[i].TopLeft.x = objDet[i].nMinX;
			poPeopleDetections[i].TopLeft.y = y_start;
			poPeopleDetections[i].BottomRight.x = objDet[i].nMaxX;
			poPeopleDetections[i].BottomRight.y = y_end;

			poPeopleDetections[i].nBody = UPPER_BODY;
			int16_t nXCentroid = (objDet[i].nMinX + objDet[i].nMaxX) >> 1;
			int16_t nYCentroid = (y_start + y_end) >> 1;
			uint16_t nAvgDepth = ComputeAverageDepth(pPointCloudImage, inpWidth, inpHeight, &nXCentroid, &nYCentroid);
			poPeopleDetections[i].nLocation.x = nXCentroid;
			poPeopleDetections[i].nLocation.y = nYCentroid;
			poPeopleDetections[i].nLocation.z = nAvgDepth;
			poPeopleDetections[i].nLabelId = i;
			poPeopleDetections[i].bMotionPresent = false;
			poPeopleDetections[i].bPredicted = false;
			poPeopleDetections[i].nTrajectoryPoints = 0;
		}
		numPeople = numPeopleDetected;
	}
	else
	{
		
		numPeople = nFilteredPeopleCount;
	}

	numPplDetected = numPeople;

	delete[] PeopleFeatures;
	return poPeopleDetections;
}

DetectedPeopleInfo* PeopleDetector::DetectStaticPeople(int& numPeople)
{
	//This function gets all the people detected who are static

	DetectedObjectInfo objDet[MAX_DETECTED_OBJECTS];
	int32_t numPeopleDetected = pobjDet->GetDetections(objDet, PERSON_CLASS, PEOPLE_DETECTION_THRESHOLD);
	numPeopleDetected = numPeopleDetected < MAX_PEOPLE_DETECTION ? numPeopleDetected : MAX_PEOPLE_DETECTION;

	ObjFeat* PeopleFeatures = new ObjFeat[numPeopleDetected];
	TrackedObj* trkObjs;
	int nFilteredPeopleCount = 0;
	if (bEnableTrackPeople)
	{
		for (int i = 0; i < numPeopleDetected; i++)
		{
			//aspect ratio changes as per Raka
			int y_start, y_end;
			float ratio1 = (objDet[i].nMaxX - objDet[i].nMinX)
				/ (float)(objDet[i].nMaxY - objDet[i].nMinY);
			float ratio2 = 1 / ratio1;
			if (!bEnableFullBodyTrack && ratio1 <= 0.7)
			{
				y_start = objDet[i].nMinY;
				y_end = ((objDet[i].nMaxY - objDet[i].nMinY) * 6)/10 + y_start;
			}
			else
			{
				y_start = objDet[i].nMinY;
				y_end = objDet[i].nMaxY;
			}

			PeopleFeatures[i].min_x = objDet[i].nMinX;
			PeopleFeatures[i].min_y = y_start;
			PeopleFeatures[i].max_x = objDet[i].nMaxX;
			PeopleFeatures[i].max_y = y_end;

			PeopleFeatures[i].area = (PeopleFeatures[i].max_x - PeopleFeatures[i].min_x) *
				(PeopleFeatures[i].max_y - PeopleFeatures[i].min_y);
			PeopleFeatures[i].centroid_x = (PeopleFeatures[i].max_x + PeopleFeatures[i].min_x) / 2 * PeopleFeatures[i].area;
			PeopleFeatures[i].centroid_y = (PeopleFeatures[i].max_y + PeopleFeatures[i].min_y) / 2 * PeopleFeatures[i].area;
			PeopleFeatures[i].ptr = NULL;
			PeopleFeatures[i].detection_type = DETECTED;
		}

		PeopleTracker->SetDepthImage(pDepthImage, pPointCloudImage);
		PeopleTracker->Process(PeopleFeatures, numPeopleDetected, frame_num);
		PeopleTracker->GetTrackedObjects(&trkObjs, numPplTracked);

		for (int i = 0; i < numPplTracked; i++)
		{
			//check if no motion 
			//verify that there is no motion present
			//do not add if track is less than required frames
			if (trkObjs[i].track_count < params.timeToStartCounting)
			{
				continue;
			}

			//how to detect motion for the tracked objects?
			//check in the last N frames if there was any motion greater than the threshold
			//if yes dont add to tracking
			bool bIsMotionPresent = false;
			int nCurrentIndex = trkObjs[i].trkTrajectoryIndx;
			int nPreviousIndex = nCurrentIndex - 1 > 0 ? nCurrentIndex - 1 : NUM_TRACK_POINTS - 1;

			//check distance travelled between last n frames to check
			for (int nTrackId = 0; nTrackId < params.timeToStartCounting; nTrackId++)
			{
				float nDist = CheckDistanceTravelled(
					trkObjs[i].PointCloudTrajectoryX[nCurrentIndex],
					trkObjs[i].PointCloudTrajectoryY[nCurrentIndex],
					trkObjs[i].PointCloudTrajectoryZ[nCurrentIndex],
					trkObjs[i].PointCloudTrajectoryX[nPreviousIndex],
					trkObjs[i].PointCloudTrajectoryY[nPreviousIndex],
					trkObjs[i].PointCloudTrajectoryZ[nPreviousIndex]);
				if (nDist > params.DistanceThresholdForStaticObjects)
				{
					bIsMotionPresent = true;
					break;
				}
				nPreviousIndex = nPreviousIndex <= 0 ? NUM_TRACK_POINTS - 1 : nPreviousIndex - 1;
			}

			poPeopleDetections[nFilteredPeopleCount].bMotionPresent = false;
			if (bIsMotionPresent == true)
			{
				poPeopleDetections[nFilteredPeopleCount].bMotionPresent = true;
			}

			poPeopleDetections[nFilteredPeopleCount].TopLeft.x = trkObjs[i].min_x;
			poPeopleDetections[nFilteredPeopleCount].TopLeft.y = trkObjs[i].min_y;
			poPeopleDetections[nFilteredPeopleCount].BottomRight.x = trkObjs[i].max_x;
			poPeopleDetections[nFilteredPeopleCount].BottomRight.y = trkObjs[i].max_y;
			poPeopleDetections[nFilteredPeopleCount].nBody = UPPER_BODY;
			int16_t nXCentroid = (trkObjs[i].min_x + trkObjs[i].max_x) >> 1;
			int16_t nYCentroid = (trkObjs[i].min_y + trkObjs[i].max_y) >> 1;
			uint16_t nAvgDepth = ComputeAverageDepth(pPointCloudImage, inpWidth, inpHeight, &nXCentroid, &nYCentroid);
			poPeopleDetections[nFilteredPeopleCount].nLocation.x = nXCentroid;
			poPeopleDetections[nFilteredPeopleCount].nLocation.y = nYCentroid;
			poPeopleDetections[nFilteredPeopleCount].nLocation.z = nAvgDepth;
			poPeopleDetections[nFilteredPeopleCount].nLabelId = trkObjs[i].track_label;
			poPeopleDetections[nFilteredPeopleCount].bPredicted = false;
			//Update tracking trajectory
			nCurrentIndex = trkObjs[i].trkTrajectoryIndx;
			int nTrajecPoints = 0;
			for (int nTrackIndex = 0;
				nTrackIndex < min(trkObjs[i].track_count, NUM_TRACK_POINTS);
				nTrackIndex++)
			{
				poPeopleDetections[nFilteredPeopleCount].o2DTrajectory[nTrackIndex].x
					= trkObjs[i].trkTrajectoryX[nCurrentIndex];
				poPeopleDetections[nFilteredPeopleCount].o2DTrajectory[nTrackIndex].y
					= trkObjs[i].trkTrajectoryY[nCurrentIndex];

				poPeopleDetections[nFilteredPeopleCount].o3DTrajectory[nTrackIndex].x
					= trkObjs[i].PointCloudTrajectoryX[nCurrentIndex];
				poPeopleDetections[nFilteredPeopleCount].o3DTrajectory[nTrackIndex].y
					= trkObjs[i].PointCloudTrajectoryY[nCurrentIndex];
				poPeopleDetections[nFilteredPeopleCount].o3DTrajectory[nTrackIndex].z
					= trkObjs[i].PointCloudTrajectoryZ[nCurrentIndex];

				nCurrentIndex--;
				if (nCurrentIndex < 0)
				{
					nCurrentIndex = NUM_TRACK_POINTS - 1;
				}
				nTrajecPoints++;
			}
			poPeopleDetections[nFilteredPeopleCount].nTrajectoryPoints = nTrajecPoints;
			nFilteredPeopleCount += 1;
		}
	}

	if (!bEnableTrackPeople)
	{
		for (int i = 0; i < numPeopleDetected; i++)
		{
			//aspect ratio changes as per Raka
			int y_start, y_end;
			float ratio1 = (objDet[i].nMaxX - objDet[i].nMinX)
				/ (float)(objDet[i].nMaxY - objDet[i].nMinY);
			float ratio2 = 1 / ratio1;
			if (!bEnableFullBodyTrack && ratio1 <= 0.7)
			{
				y_start = objDet[i].nMinY;
				y_end = ((objDet[i].nMaxY - objDet[i].nMinY) * 6)/10 + y_start;
			}
			else
			{
				y_start = objDet[i].nMinY;
				y_end = objDet[i].nMaxY;
			}

			poPeopleDetections[i].TopLeft.x = objDet[i].nMinX;
			poPeopleDetections[i].TopLeft.y = y_start;
			poPeopleDetections[i].BottomRight.x = objDet[i].nMaxX;
			poPeopleDetections[i].BottomRight.y = y_end;

			poPeopleDetections[i].nBody = UPPER_BODY;
			int16_t nXCentroid = (objDet[i].nMinX + objDet[i].nMaxX) >> 1;
			int16_t nYCentroid = (y_start + y_end) >> 1;
			uint16_t nAvgDepth = ComputeAverageDepth(pPointCloudImage, inpWidth, inpHeight, &nXCentroid, &nYCentroid);
			poPeopleDetections[i].nLocation.x = nXCentroid;
			poPeopleDetections[i].nLocation.y = nYCentroid;
			poPeopleDetections[i].nLocation.z = nAvgDepth;
			poPeopleDetections[i].nLabelId = i;
			poPeopleDetections[i].bMotionPresent = false;
			poPeopleDetections[i].bPredicted = false;
			poPeopleDetections[i].nTrajectoryPoints = 0;
		}
		numPeople = numPeopleDetected;
	}
	else
	{

		numPeople = nFilteredPeopleCount;
	}

	numPplDetected = numPeople;

	delete[] PeopleFeatures;
	return poPeopleDetections;
}

void PeopleDetector::Run()
{
	pobjDet->Detect();
	frame_num++;
}

uint16_t* PeopleDetector::GetProcessedDepthImage()
{
	return pFilteredDepthImage;
}

uint8_t* PeopleDetector::GetProcessedIRImage()
{
	return pIRImage;
}

DetectedPeopleInfo* PeopleDetector::DetectHead(int& numPeople)
{
	//this functions returns all the head locations regardless of moving or static

	DetectedObjectInfo objDet[MAX_DETECTED_OBJECTS];
	int32_t nHeadDetected = pobjDet->GetDetections(objDet, HEAD_CLASS, HEAD_DETECTION_THRESHOLD);
	nHeadDetected = nHeadDetected < MAX_PEOPLE_DETECTION ? nHeadDetected : MAX_PEOPLE_DETECTION;

	ObjFeat* HeadFeatures = new ObjFeat[nHeadDetected];
	TrackedObj* trkObjs;
	if (bEnableTrackHead)
	{
		for (int i = 0; i < nHeadDetected; i++)
		{
			HeadFeatures[i].min_x = objDet[i].nMinX;
			HeadFeatures[i].min_y = objDet[i].nMinY;
			HeadFeatures[i].max_x = objDet[i].nMaxX;
			HeadFeatures[i].max_y = objDet[i].nMaxY;
			HeadFeatures[i].area = (HeadFeatures[i].max_x - HeadFeatures[i].min_x) *
				(HeadFeatures[i].max_y - HeadFeatures[i].min_y);
			HeadFeatures[i].centroid_x = (HeadFeatures[i].max_x + HeadFeatures[i].min_x) / 2 * HeadFeatures[i].area;
			HeadFeatures[i].centroid_y = (HeadFeatures[i].max_y + HeadFeatures[i].min_y) / 2 * HeadFeatures[i].area;
			HeadFeatures[i].ptr = NULL;
			HeadFeatures[i].detection_type = DETECTED;
		}

		HeadTracker->SetDepthImage(pDepthImage, pPointCloudImage);
		HeadTracker->Process(HeadFeatures, nHeadDetected, frame_num);
		HeadTracker->GetTrackedObjects(&trkObjs, numHeadTracked);

		for (int i = 0; i < numHeadTracked; i++)
		{
			poHeadDetections[i].TopLeft.x = trkObjs[i].min_x;
			poHeadDetections[i].TopLeft.y = trkObjs[i].min_y;
			poHeadDetections[i].BottomRight.x = trkObjs[i].max_x;
			poHeadDetections[i].BottomRight.y = trkObjs[i].max_y;
			poHeadDetections[i].nBody = UPPER_BODY;

			int16_t nXCentroid = (poHeadDetections[i].TopLeft.x
				+ poHeadDetections[i].BottomRight.x) >> 1;
			int16_t nYCentroid = (poHeadDetections[i].TopLeft.y +
				poHeadDetections[i].BottomRight.y) >> 1;

			uint16_t nAvgDepth = ComputeAverageDepth(pPointCloudImage, inpWidth, inpHeight,
				&nXCentroid, &nYCentroid);
			poHeadDetections[i].nLocation.x = nXCentroid;
			poHeadDetections[i].nLocation.y = nYCentroid;
			poHeadDetections[i].nLocation.z = nAvgDepth;
			poHeadDetections[i].nLabelId = trkObjs[i].track_label;
			poHeadDetections[i].nLinkedIndex = 0;
			poHeadDetections[i].bMotionPresent = false;
			//Update tracking trajectory
			int nCurrentIndex = trkObjs[i].trkTrajectoryIndx;
			int nTrajecPoints = 0;
			for (int nTrackIndex = 0;
				nTrackIndex < min(trkObjs[i].track_count, NUM_TRACK_POINTS);
				nTrackIndex++)
			{
				poHeadDetections[i].o2DTrajectory[nTrackIndex].x
					= trkObjs[i].trkTrajectoryX[nCurrentIndex];
				poHeadDetections[i].o2DTrajectory[nTrackIndex].y
					= trkObjs[i].trkTrajectoryY[nCurrentIndex];

				poHeadDetections[i].o3DTrajectory[nTrackIndex].x
					= trkObjs[i].PointCloudTrajectoryX[nCurrentIndex];
				poHeadDetections[i].o3DTrajectory[nTrackIndex].y
					= trkObjs[i].PointCloudTrajectoryY[nCurrentIndex];
				poHeadDetections[i].o3DTrajectory[nTrackIndex].z
					= trkObjs[i].PointCloudTrajectoryZ[nCurrentIndex];

				nCurrentIndex--;
				if (nCurrentIndex < 0)
				{
					nCurrentIndex = NUM_TRACK_POINTS - 1;
				}
				nTrajecPoints++;
			}
			poHeadDetections[i].nTrajectoryPoints = nTrajecPoints;
			poHeadDetections[i].bPredicted = false;	
		}
	}

	if (!bEnableTrackPeople)
	{
		for (int i = 0; i < nHeadDetected; i++)
		{
			poHeadDetections[i].TopLeft.x = objDet[i].nMinX;
			poHeadDetections[i].TopLeft.y = objDet[i].nMinY;
			poHeadDetections[i].BottomRight.x = objDet[i].nMaxX;
			poHeadDetections[i].BottomRight.y = objDet[i].nMaxY;
			poHeadDetections[i].nBody = UPPER_BODY;

			int16_t nXCentroid = (poHeadDetections[i].TopLeft.x
				+ poHeadDetections[i].BottomRight.x) >> 1;
			int16_t nYCentroid = (poHeadDetections[i].TopLeft.y +
				poHeadDetections[i].BottomRight.y) >> 1;

			uint16_t nAvgDepth = ComputeAverageDepth(pPointCloudImage, inpWidth, inpHeight,
				&nXCentroid, &nYCentroid);
			poHeadDetections[i].nLocation.x = nXCentroid;
			poHeadDetections[i].nLocation.y = nYCentroid;
			poHeadDetections[i].nLocation.z = nAvgDepth;
			poHeadDetections[i].nLabelId = i;
			poHeadDetections[i].bMotionPresent = false;
			poHeadDetections[i].bPredicted = false;
			poHeadDetections[i].nTrajectoryPoints = 0;
		}
		numPeople = nHeadDetected;
	}
	else
	{

		numPeople = numHeadTracked;
	}

	numHeadDetected = numPeople;
	delete[] HeadFeatures;
	return poHeadDetections;
}

void PeopleDetector::GetLocationHead(int& numHead, int &numPeopleDet,
	DetectedPeopleInfo* oPeople, DetectedPeopleInfo* oHead)
{
	//get all head detections of objects within body
	//body should be static
	//if no head detection predict
	//no dependency on any other api

	//get body detections
	DetectedObjectInfo PeopleobjDet[MAX_DETECTED_OBJECTS];
	int32_t numPeopleDetected = pobjDet->GetDetections(PeopleobjDet, PERSON_CLASS, PEOPLE_DETECTION_THRESHOLD);
	numPeopleDetected = numPeopleDetected < MAX_PEOPLE_DETECTION ? numPeopleDetected : MAX_PEOPLE_DETECTION;

	ObjFeat* PeopleFeatures = new ObjFeat[numPeopleDetected];
	TrackedObj* PeopletrkObjs;
	int nFilteredPeopleCount = 0;

	for (int i = 0; i < numPeopleDetected; i++)
	{
		//aspect ratio changes as per Raka
		int y_start, y_end;
		float ratio1 = (PeopleobjDet[i].nMaxX - PeopleobjDet[i].nMinX) 
			/ (float)(PeopleobjDet[i].nMaxY - PeopleobjDet[i].nMinY);
		float ratio2 = 1 / ratio1;
		if (!bEnableFullBodyTrack && ratio1 <= 0.7)
		{
			y_start = PeopleobjDet[i].nMinY;
			y_end = ((PeopleobjDet[i].nMaxY - PeopleobjDet[i].nMinY) * 6)/10 + y_start;
		}
		else
		{
			y_start = PeopleobjDet[i].nMinY;
			y_end = PeopleobjDet[i].nMaxY;
		}

		PeopleFeatures[i].min_x = PeopleobjDet[i].nMinX;
		PeopleFeatures[i].min_y = y_start;
		PeopleFeatures[i].max_x = PeopleobjDet[i].nMaxX;
		PeopleFeatures[i].max_y = y_end;
		PeopleFeatures[i].area = (PeopleFeatures[i].max_x - PeopleFeatures[i].min_x) *
			(PeopleFeatures[i].max_y - PeopleFeatures[i].min_y);
		PeopleFeatures[i].centroid_x = (PeopleFeatures[i].max_x + PeopleFeatures[i].min_x) / 2 * PeopleFeatures[i].area;
		PeopleFeatures[i].centroid_y = (PeopleFeatures[i].max_y + PeopleFeatures[i].min_y) / 2 * PeopleFeatures[i].area;
		PeopleFeatures[i].ptr = NULL;
		PeopleFeatures[i].detection_type = DETECTED;
	}

	PeopleTracker->SetDepthImage(pDepthImage, pPointCloudImage);
	PeopleTracker->Process(PeopleFeatures, numPeopleDetected, frame_num);
	PeopleTracker->GetTrackedObjects(&PeopletrkObjs, numPplTracked);

	for (int i = 0; i < numPplTracked; i++)
	{
		oPeople[nFilteredPeopleCount].TopLeft.x = PeopletrkObjs[i].min_x;
		oPeople[nFilteredPeopleCount].TopLeft.y = PeopletrkObjs[i].min_y;
		oPeople[nFilteredPeopleCount].BottomRight.x = PeopletrkObjs[i].max_x;
		oPeople[nFilteredPeopleCount].BottomRight.y = PeopletrkObjs[i].max_y;
		oPeople[nFilteredPeopleCount].nBody = UPPER_BODY;
		oPeople[nFilteredPeopleCount].nLabelId = PeopletrkObjs[i].track_label;

		int16_t nXCentroid = (oPeople[nFilteredPeopleCount].TopLeft.x
			+ oPeople[nFilteredPeopleCount].BottomRight.x) >> 1;
		int16_t nYCentroid = (oPeople[nFilteredPeopleCount].TopLeft.y +
			oPeople[nFilteredPeopleCount].BottomRight.y) >> 1;

		uint16_t nAvgDepth = ComputeAverageDepth(pPointCloudImage, inpWidth, inpHeight,
			&nXCentroid, &nYCentroid);
		oPeople[nFilteredPeopleCount].nLocation.x = nXCentroid;
		oPeople[nFilteredPeopleCount].nLocation.y = nYCentroid;
		oPeople[nFilteredPeopleCount].nLocation.z = nAvgDepth;
		oPeople[nFilteredPeopleCount].bMotionPresent = false;
		oPeople[nFilteredPeopleCount].bPredicted = false;
		//Update tracking trajectory
		int nCurrentIndex = PeopletrkObjs[i].trkTrajectoryIndx;
		int nTrajecPoints = 0;
		for (int nTrackIndex = 0;
			nTrackIndex < min(PeopletrkObjs[i].track_count, NUM_TRACK_POINTS);
			nTrackIndex++)
		{
			oPeople[nFilteredPeopleCount].o2DTrajectory[nTrackIndex].x
				= PeopletrkObjs[i].trkTrajectoryX[nCurrentIndex];
			oPeople[nFilteredPeopleCount].o2DTrajectory[nTrackIndex].y
				= PeopletrkObjs[i].trkTrajectoryY[nCurrentIndex];

			oPeople[nFilteredPeopleCount].o3DTrajectory[nTrackIndex].x
				= PeopletrkObjs[i].PointCloudTrajectoryX[nCurrentIndex];
			oPeople[nFilteredPeopleCount].o3DTrajectory[nTrackIndex].y
				= PeopletrkObjs[i].PointCloudTrajectoryY[nCurrentIndex];
			oPeople[nFilteredPeopleCount].o3DTrajectory[nTrackIndex].z
				= PeopletrkObjs[i].PointCloudTrajectoryZ[nCurrentIndex];

			nCurrentIndex--;
			if (nCurrentIndex < 0)
			{
				nCurrentIndex = NUM_TRACK_POINTS - 1;
			}
			nTrajecPoints++;
		}
		oPeople[nFilteredPeopleCount].nTrajectoryPoints = nTrajecPoints;
		nFilteredPeopleCount += 1;
	}
	numPeopleDet = nFilteredPeopleCount;

	//get head detections
	DetectedObjectInfo HeadobjDet[MAX_DETECTED_OBJECTS];
	int32_t nHeadDetected = pobjDet->GetDetections(HeadobjDet, HEAD_CLASS, HEAD_DETECTION_THRESHOLD);
	nHeadDetected = nHeadDetected < MAX_PEOPLE_DETECTION ? nHeadDetected : MAX_PEOPLE_DETECTION;

	int nHeadAssosiated = 0;

	//match body detections with head detections 
	//and extrapolate objects which do not match
	ObjFeat* HeadFeatures = new ObjFeat[numPeopleDet];

	//for each of the body, see which head fits best
	for (int j = 0; j < numPeopleDet; j++)
	{
		//remove previous linkage
		oPeople[j].nLinkedIndex = -1;

		//find 1st head that lies within body
		for (int i = 0; i < nHeadDetected; i++)
		{
			//same head cannot lie within 2 bodies so we should be fine
			//if 2 bodies overlap iou threshold will remove it

			if (HeadobjDet[i].nMinX < oPeople[j].TopLeft.x)
			{
				continue;
			}

			if (HeadobjDet[i].nMinY < oPeople[j].TopLeft.y)
			{
				continue;
			}

			if (HeadobjDet[i].nMaxX > oPeople[j].BottomRight.x)
			{
				continue;
			}

			if (HeadobjDet[i].nMaxY > oPeople[j].BottomRight.y)
			{
				continue;
			}		

			//update object feature
			HeadFeatures[j].min_x = HeadobjDet[i].nMinX;
			HeadFeatures[j].min_y = HeadobjDet[i].nMinY;
			HeadFeatures[j].max_x = HeadobjDet[i].nMaxX;
			HeadFeatures[j].max_y = HeadobjDet[i].nMaxY;
			HeadFeatures[j].area = (HeadFeatures[j].max_x - HeadFeatures[j].min_x) *
				(HeadFeatures[j].max_y - HeadFeatures[j].min_y);
			HeadFeatures[j].centroid_x = (HeadFeatures[j].max_x + HeadFeatures[j].min_x)
				/ 2 * HeadFeatures[j].area;
			HeadFeatures[j].centroid_y = (HeadFeatures[j].max_y + HeadFeatures[j].min_y)
				/ 2 * HeadFeatures[j].area;
			HeadFeatures[j].ptr = NULL;
			HeadFeatures[j].detection_type = DETECTED;//detected	
			oPeople[j].nLinkedIndex = i;
			break;
		}

		//On faces that are not detected, we can predict using approximate location in head
		if (oPeople[j].nLinkedIndex == -1)
		{
			//update object feature
			//approximate head location to top 2/3 of body
			int Xmin = 0;
			int Xmax = 0;
			int Ymin = 0;
			int Ymax = 0;
#if 0
			uint16_t nXLocation = (oPeople[j].TopLeft.x + oPeople[j].BottomRight.x) >> 1;
			uint16_t nYLocation = (oPeople[j].TopLeft.y + oPeople[j].BottomRight.y) >> 1;

			//head is approximate 1/4 of upper body
			//note:these values are chosen based on observation, change this if it doesnt match with visual observation
			uint16_t nHeadWidth = (oPeople[j].BottomRight.x - oPeople[j].TopLeft.x) / 4;
			uint16_t nHeadLength = (oPeople[j].BottomRight.y - oPeople[j].TopLeft.y) / 6;

			//use depth to find better location of head, find which block is closest to the body pixels
			FindHeadLocationUsingDepth(pDepthImage, inpWidth, inpHeight,
				&nXLocation,
				&nYLocation,
				nHeadWidth,
				nHeadLength,
				oPeople[j].TopLeft.x,
				oPeople[j].BottomRight.x,
				oPeople[j].TopLeft.y,
				WINDOW_SEARCH_STRIDE);

			Xmin = nXLocation - nHeadWidth / 2;
			Xmax = nXLocation + nHeadWidth / 2;
			Ymin = nYLocation;
			Ymax = nYLocation + nHeadLength;
#else
			DetectedObjectInfo oPeopleObj;
			oPeopleObj.nMinX = oPeople[j].TopLeft.x;
			oPeopleObj.nMinY = oPeople[j].TopLeft.y;
			oPeopleObj.nMaxX = oPeople[j].BottomRight.x;
			oPeopleObj.nMaxY = oPeople[j].BottomRight.y;
			DetectedObjectInfo oPeoplePred = GetHeadFromRoi(oPeopleObj);
			Xmin = oPeoplePred.nMinX;
			Xmax = oPeoplePred.nMaxX;
			Ymin = oPeoplePred.nMinY;
			Ymax = oPeoplePred.nMaxY;
#endif
			HeadFeatures[j].min_x = Xmin;
			HeadFeatures[j].min_y = Ymin;
			HeadFeatures[j].max_x = Xmax;
			HeadFeatures[j].max_y = Ymax;
			HeadFeatures[j].area = (HeadFeatures[j].max_x - HeadFeatures[j].min_x) *
				(HeadFeatures[j].max_y - HeadFeatures[j].min_y);
			HeadFeatures[j].centroid_x = (HeadFeatures[j].max_x + HeadFeatures[j].min_x)
				/ 2 * HeadFeatures[j].area;
			HeadFeatures[j].centroid_y = (HeadFeatures[j].max_y + HeadFeatures[j].min_y)
				/ 2 * HeadFeatures[j].area;
			HeadFeatures[j].ptr = NULL;
			HeadFeatures[j].detection_type = PREDICTED;	
		}
	}
		
	TrackedObj* HeadtrkObjs;
	HeadTracker->SetDepthImage(pDepthImage, pPointCloudImage);
	HeadTracker->Process(HeadFeatures, numPeopleDet, frame_num);
	HeadTracker->GetTrackedObjects(&HeadtrkObjs, numHeadTracked);

	int nPixelsThreshold = 100;
	int nHeadTrackedFiltered = 0;

	for (int k = 0; k < numPeopleDet; k++)
	{
		oPeople[k].nLinkedIndex = -1;
	}

	for (int i = 0; i < numHeadTracked; i++)
	{
		//set index to unlinked
		oHead[i].nLinkedIndex = -1;

		for (int j = 0; j < numPeopleDet; j++)
		{
			//check if this body is already linked to a head.
			//used to remove duplicate detection on heads within same body
			if (oPeople[j].nLinkedIndex != -1)
			{
				continue;
			}

			/*
			if (HeadtrkObjs[i].min_x + nPixelsThreshold < oPeople[j].TopLeft.x)
			{
				continue;
			}

			if (HeadtrkObjs[i].min_y + nPixelsThreshold < oPeople[j].TopLeft.y)
			{
				continue;
			}

			if (HeadtrkObjs[i].max_x - nPixelsThreshold > oPeople[j].BottomRight.x)
			{
				continue;
			}

			if (HeadtrkObjs[i].max_y - nPixelsThreshold > oPeople[j].BottomRight.y)
			{
				continue;
			}
			*/

			//verify that head lies within body
			//nPixelsThreshold added because sometimes the predicted object is above the box
			if (HeadtrkObjs[i].min_x + nPixelsThreshold >= oPeople[j].TopLeft.x &&
				HeadtrkObjs[i].min_y + nPixelsThreshold >= oPeople[j].TopLeft.y &&
				HeadtrkObjs[i].max_x <= oPeople[j].BottomRight.x &&
				HeadtrkObjs[i].max_y <= oPeople[j].BottomRight.y)
			{
				//remove based on size, not implemented yet
				int nHeadWid = HeadtrkObjs[i].max_x - HeadtrkObjs[i].min_x;
				int nHeadLen = HeadtrkObjs[i].max_y - HeadtrkObjs[i].min_y;

				int nBodyWid = oPeople[j].BottomRight.x - oPeople[j].TopLeft.x;
				int nBodyLen = oPeople[j].BottomRight.y - oPeople[j].TopLeft.y;

				/*
				//TODO:0.2 might not work in all cases
				if (nHeadWid > 0.2 * nBodyWid)
				{
					continue;
				}

				if (nHeadLen > 0.2 * nBodyLen)
				{
					continue;
				}
				*/

				//verify that there is no motion present
				//do not add if track is less than required frames
				if (PeopletrkObjs[j].track_count < params.timeToStartCounting)
				{
					continue;
				}

				//how to detect motion for the tracked objects?
				//check in the last N frames if there was any motion greater than the threshold
				//if yes dont add to tracking
				bool bIsMotionPresent = false;
				int nCurrentIndex = PeopletrkObjs[j].trkTrajectoryIndx;
				int nPreviousIndex = nCurrentIndex - 1 > 0 ? nCurrentIndex - 1 : NUM_TRACK_POINTS - 1;

				//check distance travelled between last n frames to check
				for (int nTrackId = 0; nTrackId < params.timeToStartCounting; nTrackId++)
				{
					float nDist = CheckDistanceTravelled(
						PeopletrkObjs[j].PointCloudTrajectoryX[nCurrentIndex],
						PeopletrkObjs[j].PointCloudTrajectoryY[nCurrentIndex],
						PeopletrkObjs[j].PointCloudTrajectoryZ[nCurrentIndex],
						PeopletrkObjs[j].PointCloudTrajectoryX[nPreviousIndex],
						PeopletrkObjs[j].PointCloudTrajectoryY[nPreviousIndex],
						PeopletrkObjs[j].PointCloudTrajectoryZ[nPreviousIndex]);
					if (nDist > params.DistanceThresholdForStaticObjects)
					{
						bIsMotionPresent = true;
						break;
					}
					nPreviousIndex = nPreviousIndex <= 0 ? NUM_TRACK_POINTS - 1 : nPreviousIndex - 1;
				}

#ifdef _DEBUG
				oHead[nHeadTrackedFiltered].bMotionPresent = false;
				if (bIsMotionPresent == true)
				{
					oHead[nHeadTrackedFiltered].bMotionPresent = true;
				}
#else
				if (bIsMotionPresent == true)
				{
					//in release mode we will not send out these objects to 
					//avoid confusion
					continue;
				}
				oHead[nHeadTrackedFiltered].bMotionPresent = false;
#endif

				//populate final tracked object
				oHead[nHeadTrackedFiltered].TopLeft.x = HeadtrkObjs[i].min_x;
				oHead[nHeadTrackedFiltered].TopLeft.y = HeadtrkObjs[i].min_y;
				oHead[nHeadTrackedFiltered].BottomRight.x = HeadtrkObjs[i].max_x;
				oHead[nHeadTrackedFiltered].BottomRight.y = HeadtrkObjs[i].max_y;
				oHead[nHeadTrackedFiltered].nBody = UPPER_BODY;

				int16_t nXCentroid = (oHead[nHeadTrackedFiltered].TopLeft.x
					+ oHead[nHeadTrackedFiltered].BottomRight.x) >> 1;
				int16_t nYCentroid = (oHead[nHeadTrackedFiltered].TopLeft.y +
					oHead[nHeadTrackedFiltered].BottomRight.y) >> 1;

				uint16_t nAvgDepth = ComputeAverageDepth(pPointCloudImage, inpWidth, inpHeight,
					&nXCentroid, &nYCentroid);
				oHead[nHeadTrackedFiltered].nLocation.x = nXCentroid;
				oHead[nHeadTrackedFiltered].nLocation.y = nYCentroid;
				oHead[nHeadTrackedFiltered].nLocation.z = nAvgDepth;
#ifdef _DEBUG
				oHead[nHeadTrackedFiltered].bPredicted = HeadtrkObjs[i].bPredicted;
#else
				oHead[nHeadTrackedFiltered].bPredicted = false;
#endif				
				//using label of people so even if new head track is formed id doesnt change
				oHead[nHeadTrackedFiltered].nLabelId = oPeople[j].nLabelId;// HeadtrkObjs[i].track_label;
				oHead[nHeadTrackedFiltered].nLinkedIndex = j;
				oPeople[j].nLinkedIndex = nHeadTrackedFiltered;

				//Update tracking trajectory
				nCurrentIndex = HeadtrkObjs[i].trkTrajectoryIndx;
				int nTrajecPoints = 0;
				for (int nTrackIndex = 0;
					nTrackIndex < min(HeadtrkObjs[i].track_count, NUM_TRACK_POINTS);
					nTrackIndex++)
				{
					oHead[nHeadTrackedFiltered].o2DTrajectory[nTrackIndex].x
						= HeadtrkObjs[i].trkTrajectoryX[nCurrentIndex];
					oHead[nHeadTrackedFiltered].o2DTrajectory[nTrackIndex].y
						= HeadtrkObjs[i].trkTrajectoryY[nCurrentIndex];

					oHead[nHeadTrackedFiltered].o3DTrajectory[nTrackIndex].x
						= HeadtrkObjs[i].PointCloudTrajectoryX[nCurrentIndex];
					oHead[nHeadTrackedFiltered].o3DTrajectory[nTrackIndex].y
						= HeadtrkObjs[i].PointCloudTrajectoryY[nCurrentIndex];
					oHead[nHeadTrackedFiltered].o3DTrajectory[nTrackIndex].z
						= HeadtrkObjs[i].PointCloudTrajectoryZ[nCurrentIndex];

					nCurrentIndex--;
					if (nCurrentIndex < 0)
					{
						nCurrentIndex = NUM_TRACK_POINTS - 1;
					}
					nTrajecPoints++;
				}
				oHead[nHeadTrackedFiltered].nTrajectoryPoints = nTrajecPoints;
				nHeadTrackedFiltered += 1;
				break;
			}		
		}
	}

	numHead = nHeadTrackedFiltered;
	delete[] HeadFeatures;
	delete[] PeopleFeatures;
	return;
}

void PeopleDetector::SegmentUsingDepthRange(DetectedObjectInfo* objDet)
{
	int y_start;
	int y_end;
	int depth_label = 20;
	roi roiparams;
	memset(pLabelImageForSilloheute, 0, inpWidth * inpHeight * sizeof(uint8_t));

	for (int i = 0; i < numPplDetected; i++)
	{
		/* get the aspect ratio of the bounding box */
		float ratio1 = (objDet[i].nMaxX - objDet[i].nMinX) / (float)(objDet[i].nMaxY - objDet[i].nMinY);
		float ratio2 = 1 / ratio1;
		if (!bEnableFullBodyTrack && ratio1 <= 0.7)
		{
			y_start = objDet[i].nMinY;
			y_end = ((objDet[i].nMaxY - objDet[i].nMinY) * 6)/10 + y_start;
		}
		else
		{
			y_start = objDet[i].nMinY;
			y_end = objDet[i].nMaxY;
		}
		//y_start = objDet[i].nMinY;
		//y_end = objDet[i].nMaxY;
		//cout << objDet[i].nMinY << " " << objDet[i].nMaxY << endl;
		//cout << y_start << " " << y_end << endl;

		int centroid_x = (objDet[i].nMaxX + objDet[i].nMinX) / 2;
		int centroid_y = (y_end + y_start) / 2;

		roiparams.minx = objDet[i].nMinX;
		roiparams.miny = y_start;
		roiparams.maxx = objDet[i].nMaxX;
		roiparams.maxy = y_end;

		regionGrow_int16(pDepthImage, pLabelImageForSilloheute,
			inpHeight, inpWidth,
			roiparams,
			centroid_x,
			centroid_y,
			15);

		for (int y = y_start; y < y_end; y++)
		{
			for (int x = objDet[i].nMinX; x < objDet[i].nMaxX; x++)
			{
				if (pDepthImage[y * inpWidth + x] != 0)
				{
					pLabelImageForSilloheute[y * inpWidth + x] = depth_label;
				}
			}
		}

		depth_label += 50;
		if (depth_label > 2048) depth_label = 20;
	}
}

void PeopleDetector::SegmentUsingMeanandVariance(DetectedObjectInfo* objDet)
{
	float mean;
	float sum = 0;
	float std_dev;
	int count = 0;
	for (int i = 0; i < numPplDetected; i++)
	{
		for (int y = objDet[i].nMinY; y < objDet[i].nMaxY; y++)
		{
			for (int x = objDet[i].nMinX; x < objDet[i].nMaxX; x++)
			{
				if (pDepthImage[y * inpWidth + x] != 0)
				{
					sum += pDepthImage[y * inpWidth + x];
					count++;
				}
			}
		}

		if (count != 0)
		{
			mean = sum / count;
		}
		else
		{
			mean = sum;
		}
		sum = 0;
		for (int y = objDet[i].nMinY; y < objDet[i].nMaxY; y++)
		{
			for (int x = objDet[i].nMinX; x < objDet[i].nMaxX; x++)
			{
				if (pDepthImage[y * inpWidth + x] != 0)
					sum += (pDepthImage[y * inpWidth + x] - mean) * (pDepthImage[y * inpWidth + x] - mean);
			}
		}
		if (count != 0)
		{
			std_dev = sqrtf(sum / count);
		}
		else
		{
			std_dev = sqrtf(sum);
		}
		
		for (int y = objDet[i].nMinY; y < objDet[i].nMaxY; y++)
		{
			for (int x = objDet[i].nMinX; x < objDet[i].nMaxX; x++)
			{
				if (pDepthImage[y * inpWidth + x] != 0)
				{
					if ((pDepthImage[y * inpWidth + x]) - mean < 0.1 * std_dev)
					{
						pDepthImage[y * inpWidth + x] = 0;
					}
				}
			}
		}
	}
}

void ThresholdDepthImageOnROI(uint16_t* pDepthImage, uint16_t* pSilhouetteImage, int16_t nRows, int16_t nCols, DetectedObjectInfo* pBBox, int index, int16_t nDepthThreshold, int16_t nCenDepth)
{	
	int width = pBBox[index].nMaxX - pBBox[index].nMinX;
	for (int nY = pBBox[index].nMinY, ty = 0; nY < pBBox[index].nMaxY; nY++, ty++)
	{
		for (int nX = pBBox[index].nMinX, tx = 0; nX < pBBox[index].nMaxX; nX++, tx++)
		{
			if (abs(pDepthImage[nY * nCols + nX] - nCenDepth) < nDepthThreshold)
			{
				pSilhouetteImage[ty * width + tx] = 1;
			}
		}
	}
}

cv::Point GetCentroidOfMaxObjectOnROI(cv::Mat thr, int16_t nRows, 
	int16_t nCols, DetectedObjectInfo* pBBox)
{
	cv::Point nCentroid;
	//cv::Mat thr = Mat::zeros(nCols, nRows , CV_8UC1);
	// Thr variable is of type uint8 as connectedComponentsWithStats needs 8 bit mat file as input
	Mat labelImage(nCols, nRows, CV_32S);
	Mat stats, centroids;
	int nLabels = connectedComponentsWithStats(thr, labelImage, stats, centroids, 8, CV_32S);

	int max_area = -1, max_index = 0;
	for (int label = 0; label < nLabels; ++label)
	{
		if ((stats.at<int>(label, CC_STAT_AREA)) > max_area)
		{
			max_index = label;
			max_area = stats.at<int>(label, CC_STAT_AREA);
		}
	}
	/*
	int min_x, min_y, max_x, max_y;
	min_x = stats.at<int>(max_index, CC_STAT_LEFT);
	min_y = stats.at<int>(max_index, CC_STAT_TOP);
	max_x = min_x + stats.at<int>(max_index, CC_STAT_WIDTH);
	max_y = min_y + stats.at<int>(max_index, CC_STAT_HEIGHT);
	*/
	nCentroid.x = (int)centroids.at<double>(max_index, 0);
	nCentroid.y = (int)centroids.at<double>(max_index, 1);
	return nCentroid;
}

cv::Point GetMaximumHeadPointOnSilloheutteOnROI(cv::Mat thr, DetectedObjectInfo* pBBox, int head_width, cv::Mat drawing)
{
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find Contours
	findContours(thr, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

	if (contours.size() == 0) return cv::Point(0, 0);
	vector< vector<Point> > hull(contours.size());
	for (int i = 0; i < contours.size(); i++)
		convexHull(Mat(contours[i]), hull[i], false);

	double maxArea = 0;
	int maxAreaContourId = 0;
	for (int j = 0; j < contours.size(); j++) {
		double newArea = cv::contourArea(contours.at(j));
		if (newArea > maxArea) {
			maxArea = newArea;
			maxAreaContourId = j;
		}
	}
#ifdef DEBUG_VISUALIZE
	Scalar color_contours = Scalar(0, 255, 0); // green - color for contours
	Scalar color = Scalar(255, 0, 0); // blue - color for convex hull
	drawContours(drawing, contours, maxAreaContourId, color_contours, 1, 8, vector<Vec4i>(), 0, Point());
	drawContours(drawing, hull, maxAreaContourId, color, 1, 8, vector<Vec4i>(), 0, Point());
#endif // DEBUG
	//drawContours(drawing, contours, maxAreaContourId, (0,255,0), 1, 8, vector<Vec4i>(), 0, Point());
	//drawContours(drawing, hull, maxAreaContourId, (255,0,0), 1, 8, vector<Vec4i>(), 0, Point());

	int min_x = INT_MAX, min_y = INT_MAX;
	int max_x = -1, max_y = -1;

	for (int i = 0; i < hull.at(maxAreaContourId).size(); i += 1)
	{
		if (hull.at(maxAreaContourId)[i].y < min_y) {
			min_y = hull.at(maxAreaContourId)[i].y;
			min_x = hull.at(maxAreaContourId)[i].x;
		}
	}

#ifdef DEBUG_VISUALIZE
	double contour_area = cv::contourArea(contours.at(maxAreaContourId));
	double hull_area = cv::contourArea(hull.at(maxAreaContourId));

	float solidity_ratio = (float)(hull_area / contour_area);
	//cout << "solidity_ratio =" << solidity_ratio << endl;
#endif // DEBUG_VISUALIZE

	int point_x[20], point_y[20], hull_points = 0;
	for (int i = 0; i < hull.at(maxAreaContourId).size(); i += 1)
	{
		if (hull.at(maxAreaContourId)[i].y >= min_y && hull.at(maxAreaContourId)[i].y <= min_y + 0.1 * head_width) {
			point_x[hull_points] = hull.at(maxAreaContourId)[i].x;
			point_y[hull_points] = hull.at(maxAreaContourId)[i].y;
			hull_points++;
		}
	}
	int sum_x = 0, sum_y = 0, avg_x = 0, avg_y = 0;
	for (int i = 0; i < hull_points; i++)
	{
		sum_x += point_x[i];
		sum_y += point_y[i];
	}

	if (hull_points != 0)
	{
		avg_x = (int)(sum_x / (float)hull_points);
		avg_y = (int)(sum_y / (float)hull_points);
	}
	else
	{
		// When there is no countours found 
		avg_x = 0;
		avg_y = 0;
	}
	cv::Point avg;
	avg.x = avg_x;
	avg.y = avg_y;
	return avg;
}

uint8_t* PeopleDetector::GetSilhouette()
{
	DetectedObjectInfo objDet[MAX_DETECTED_OBJECTS];
	int32_t numPeopleDetected = pobjDet->GetDetections(objDet, PERSON_CLASS, PEOPLE_DETECTION_THRESHOLD);
	numPeopleDetected = numPeopleDetected < MAX_PEOPLE_DETECTION ? numPeopleDetected : MAX_PEOPLE_DETECTION;

	DetectedObjectInfo headObjDet[MAX_DETECTED_OBJECTS];
	int32_t nHeadDetected = pobjDet->GetDetections(headObjDet, HEAD_CLASS, PEOPLE_DETECTION_THRESHOLD);
	nHeadDetected = nHeadDetected < MAX_PEOPLE_DETECTION ? nHeadDetected : MAX_PEOPLE_DETECTION;

	//memset(pSilhouette, 0, inpHeight * inpWidth);
	for (int i = 0; i < numPeopleDetected; i++)
	{
		int y_start, y_end;
		float ratio1 = (objDet[i].nMaxX - objDet[i].nMinX)
			/ (float)(objDet[i].nMaxY - objDet[i].nMinY);
		float ratio2 = 1 / ratio1;
		if (!bEnableFullBodyTrack && ratio1 <= 0.7)
		{
			y_start = objDet[i].nMinY;
			y_end = ((objDet[i].nMaxY - objDet[i].nMinY) * 6)/10 + y_start;
		}
		else
		{
			y_start = objDet[i].nMinY;
			y_end = objDet[i].nMaxY;
		}
		objDet[i].nMinY = y_start;
		objDet[i].nMaxY = y_end;

		Mat croppedSilhouette;
		bool usingHeadDetectionsFlag = false;

      	// Check for head detections and if detetected the capture silhouette referencing the head Depth
		for (int j = 0; j < nHeadDetected; j++)
		{
			if (headObjDet[j].nMinX >= objDet[i].nMinX && headObjDet[j].nMaxX <= objDet[i].nMaxX)
			{
				croppedSilhouette = PeopleDetector::GetSilhouettePeopleFromHead(headObjDet,
					objDet, i, j);
				usingHeadDetectionsFlag = true;
				break;
			}
		}
		//If no head were detected 
		if (usingHeadDetectionsFlag == false)
		{
			croppedSilhouette = PeopleDetector::GetSilhouettePeople(objDet, i);
		}

		//Mat croppedSilhouette = PeopleDetector::GetSilhouettePeople(objDet, i);
		uint16_t* detectedSil = (uint16_t*)croppedSilhouette.data;
		int cols = objDet[i].nMaxX - objDet[i].nMinX;
		for (int nY = objDet[i].nMinY, ty = 0; nY < objDet[i].nMaxY; nY++, ty++)
		{
			for (int nX = objDet[i].nMinX, tx = 0; nX < objDet[i].nMaxX; nX++, tx++)
			{
				if (detectedSil[ty * cols + tx] > 0 && pSilhouette[nY * inpWidth + nX] == 0)
					PeopleDetector::pSilhouette[nY * inpWidth + nX] = i + 1;
			}
		}

	}
	return pSilhouette;
}

void regionGrow(uint16_t* img, uint16_t* outimg, int32_t depthCols, 
	DetectedObjectInfo* roiparams, int32_t seed_x, int32_t seed_y, 
	int32_t threshold, int nLabel, int16_t referenceVal)
{
	int32_t stack_head_index = 0;
	int32_t stack_count = 0;
	int32_t neighbours[8 * 2] = { -1,-1, -1,0, -1,1, 0,-1, 0,1, 1,-1, 1,0, 1,1 };

	int32_t cols = roiparams[nLabel].nMaxX - roiparams[nLabel].nMinX;
	int32_t rows = roiparams[nLabel].nMaxY - roiparams[nLabel].nMinY;
	uint32_t* stack = new uint32_t[rows * cols];

	//stack push
	stack[stack_head_index] = (seed_x << 16) | seed_y;
	stack_count++;
	stack_head_index++;

	while (stack_count)
	{
		// stack pop
		uint16_t x_centre = (stack[stack_head_index - 1] & 0xFFFF0000) >> 16;
		uint16_t y_centre = (stack[stack_head_index - 1] & 0x0000FFFF);
		stack_head_index--;
		stack_count--;

		for (int i = 0; i < 8; i++)
		{
			int y = y_centre + neighbours[2 * i];
			int x = x_centre + neighbours[2 * i + 1];
			if (x > 0 && x < cols && y > 0 && y < rows)
			{
				if (abs(img[(roiparams[nLabel].nMinY + y) * depthCols + roiparams[nLabel].nMinX + x] - referenceVal) <= threshold && outimg[y * cols + x] == 0)
				{
					stack[stack_head_index] = (x << 16) | y;
					stack_count++;
					stack_head_index++;

					outimg[y * cols + x] = 255;
#if DEBUG_VISUALIZE
					outimg[y * cols + x] = INT_MAX;
#endif
				}
			}
		}

	}
	delete[] stack;
}

cv::Mat PeopleDetector::GetSilhouettePeopleFromHead(DetectedObjectInfo* headObjDet, DetectedObjectInfo* objDet, int index, int headIndex)
{
	int32_t nBufSzForImage = inpWidth * inpHeight * elementSize;

	Mat ganZImage(inpWidth, inpHeight, CV_16UC1);
	//depth image
	std::memcpy(ganZImage.data, pDepthImage, nBufSzForImage);

	int16_t HeadWidth = headObjDet[headIndex].nMaxX - headObjDet[headIndex].nMinX;
	int16_t HeadLength = headObjDet[headIndex].nMaxY - headObjDet[headIndex].nMinY;

	int16_t nCenY = headObjDet[headIndex].nMinY + HeadLength / 2;
	int16_t nCenX = headObjDet[headIndex].nMinX + HeadWidth / 2;
	int nBlockSize = BLOCK_SIZE;
	int16_t nDepthThr = DEPTH_THRESHOLD;

	int16_t nCenDepthHead = CalculateDepth((uint16_t*)ganZImage.data, ganZImage.cols, ganZImage.rows, nCenX, nCenY, nBlockSize, nBlockSize);
	cv::Mat subImg = ganZImage(cv::Range(objDet[index].nMinY, objDet[index].nMaxY), cv::Range(objDet[index].nMinX, objDet[index].nMaxX));

	cv::dilate(subImg, subImg, getStructuringElement(MORPH_RECT, Size(3, 3)), cv::Point(-1, -1), 1);
	cv::erode(subImg, subImg, getStructuringElement(MORPH_RECT, Size(3, 3)), cv::Point(-1, -1), 1);

	Mat tempSilhouette = Mat::zeros(subImg.size(), CV_16UC1);
	nCenY = headObjDet[headIndex].nMinY - objDet[index].nMinY + HeadLength / 2;
	nCenX = headObjDet[headIndex].nMinX - objDet[index].nMinX + HeadWidth / 2;

  	//Head Depth would be used as the reference value for region grow code
	//nCenX , nCenY are the center points of bounding box of head
	uint16_t* depth = (uint16_t*)ganZImage.data;
	regionGrow((uint16_t*)ganZImage.data, (uint16_t*)tempSilhouette.data, ganZImage.cols,
		objDet, nCenX, nCenY, nDepthThr, index, nCenDepthHead);

	return tempSilhouette;
}

cv::Mat PeopleDetector::GetSilhouettePeople(DetectedObjectInfo* objDet,int index)
{
	int32_t nBufSzForImage = inpWidth * inpHeight * elementSize;
	
	Mat ganZImage(inpWidth, inpHeight, CV_16UC1);
	//depth image
	std::memcpy(ganZImage.data, pDepthImage, nBufSzForImage);
	
	int16_t BodyWidth = objDet[index].nMaxX - objDet[index].nMinX;
	int16_t BodyLength = objDet[index].nMaxY - objDet[index].nMinY;

	int16_t nCenY = objDet[index].nMinY + BodyLength / 2;
	int16_t nCenX = objDet[index].nMinX + BodyWidth / 2;
	int nBlockSize = BLOCK_SIZE;
	int16_t nDepthThr = DEPTH_THRESHOLD;
		
	int16_t nCenDepthAvg = CalculateDepth((uint16_t*)ganZImage.data, ganZImage.cols, ganZImage.rows, nCenX, nCenY, nBlockSize , nBlockSize );
	cv::Mat subImg = ganZImage(cv::Range(objDet[index].nMinY, objDet[index].nMaxY), cv::Range(objDet[index].nMinX, objDet[index].nMaxX));
	
	cv::dilate(subImg, subImg, getStructuringElement(MORPH_RECT, Size(3, 3)), cv::Point(-1, -1), 1);
	cv::erode(subImg, subImg, getStructuringElement(MORPH_RECT, Size(3, 3)), cv::Point(-1, -1), 1);
	
	Mat tempSilhouette = Mat::zeros(subImg.size(), CV_16UC1);

	nCenY = BodyLength / 2;
	nCenX = BodyWidth / 2;
	uint16_t* depthImg = (uint16_t*)ganZImage.data;
	const int binSize = 200;
	uint16_t hist[MAX_DEPTH / binSize] = { 0 };
	int maxVal = 0;
	int maxIndex = 0;
  	// Creating Histograms using the depth
	for (int nX = objDet[index].nMinX; nX < objDet[index].nMaxX; nX++)
	{
		for (int nY = objDet[index].nMinY; nY < objDet[index].nMaxY; nY++)
		{
			if (depthImg[nY * ganZImage.cols + nX] < MAX_DEPTH && depthImg[nY * ganZImage.cols + nX] > 0)
			{
				hist[depthImg[nY * ganZImage.cols + nX] / binSize]++;
			}
		}
	}

  	//Taking 1st maximum peak from histogram which is greater than histLimit
	// The index number of histogram will give the approximate depth
	int histLimit = (9 * (objDet[index].nMaxX - objDet[index].nMinX) * (objDet[index].nMaxY - objDet[index].nMinY)) / 100;
	for (int i = 1; i < (MAX_DEPTH / binSize) - 3; i++)
	{
		if (hist[i] > maxVal && hist[i] > hist[i + 1] && hist[i] > hist[i + 2])
		{
			maxVal = hist[i];
			maxIndex = i;
		}
		if (maxVal > histLimit)
		{
			break;
		}
	}

  	// Weighted average of 2 neighbouring bins from maxima is taken to compute the center depth from histograms
	int totalHistPixels = hist[maxIndex] + hist[maxIndex - 1] + hist[maxIndex + 1];
	double tempVar = (maxIndex) * (hist[maxIndex] / (double)totalHistPixels);
	tempVar += (maxIndex + 1) * (hist[maxIndex + 1] / (double)totalHistPixels);
	tempVar += (maxIndex - 1) * (hist[maxIndex - 1] / (double)totalHistPixels);
	tempVar *= binSize;
	int16_t nCenDepthHist = (int16_t)tempVar;
	nCenX = BodyWidth / 2;
	nCenY = BodyLength / 2;

	uint32_t avgLeft = 0, avgRight = 0;
	for (int i = 0; i < 10; i++)
	{
		avgLeft += depthImg[(objDet[index].nMinY + nCenY) * ganZImage.cols + objDet[index].nMinX + nCenX - 6 - i];
		avgRight += depthImg[(objDet[index].nMinY + nCenY) * ganZImage.cols + objDet[index].nMinX + nCenX + 6 + i];
	}

	avgLeft /= 10;
	avgRight /= 10;
  
  	//If the difference between centre Depth of Histogram and averaged centre depth around the centrer of ROI is within
	// the allowable threshold depth limit then we are using the averaged center depth as the reference value
	// else using the depth computed from histograms as the reference value.

	//If we are not getting the correct seed point which has the near about value (i.e diff of reference val and the depth of seed point > nDepthThr/2)
	//then we are moving the seed point either towards left or right depending upon the difference which is less.
	// Upon finding a seed point which has depth value near about the reference value we call the region grow using the need seed point 
	if (abs(nCenDepthHist - nCenDepthAvg) < nDepthThr)
	{
		if (abs(avgLeft - nCenDepthAvg) < abs(avgRight - nCenDepthAvg))
		{
			int diff = abs(depthImg[(objDet[index].nMinY + nCenY) * ganZImage.cols + objDet[index].nMinX + nCenX] - nCenDepthAvg);
			while (diff > nDepthThr / 2)
			{
				if (nCenX > 0)
				{
					nCenX -= 1;
					diff = abs(depthImg[(objDet[index].nMinY + nCenY) * ganZImage.cols + objDet[index].nMinX + nCenX] - nCenDepthAvg);

				}
				else
				{
					nCenX = BodyWidth / 2;
					break;
				}
			}
		}
		else
		{
			int diff = abs(depthImg[(objDet[index].nMinY + nCenY) * ganZImage.cols + objDet[index].nMinX + nCenX] - nCenDepthAvg);
			while (diff > nDepthThr / 2)
			{
				if (nCenX < objDet[index].nMaxX)
				{
					nCenX += 1;
					diff = abs(depthImg[(objDet[index].nMinY + nCenY) * ganZImage.cols + objDet[index].nMinX + nCenX] - nCenDepthAvg);
				}
				else
				{
					nCenX = BodyWidth / 2;
					break;
				}
			}
		}
		regionGrow((uint16_t*)ganZImage.data, (uint16_t*)tempSilhouette.data, ganZImage.cols, objDet, nCenX, nCenY, nDepthThr, index, nCenDepthAvg);
	}
	else
	{
		if (abs(avgLeft - nCenDepthHist) < abs(avgRight - nCenDepthHist))
		{
			int diff = abs(depthImg[(objDet[index].nMinY + nCenY) * ganZImage.cols + objDet[index].nMinX + nCenX] - nCenDepthHist);
			while (diff > nDepthThr / 2)
			{
				if (nCenX > 0)
				{
					nCenX -= 1;
					diff = abs(depthImg[(objDet[index].nMinY + nCenY) * ganZImage.cols + objDet[index].nMinX + nCenX] - nCenDepthHist);

				}
				else
				{
					nCenX = BodyWidth / 2;
					break;
				}
			}
		}
		else
		{
			int diff = abs(depthImg[(objDet[index].nMinY + nCenY) * ganZImage.cols + objDet[index].nMinX + nCenX] - nCenDepthHist);
			while (diff > nDepthThr / 2)
			{
				if (nCenX < objDet[index].nMaxX)
				{
					nCenX += 1;
					diff = abs(depthImg[(objDet[index].nMinY + nCenY) * ganZImage.cols + objDet[index].nMinX + nCenX] - nCenDepthHist);
				}
				else
				{
					nCenX = BodyWidth / 2;
					break;
				}
			}
		}
		regionGrow((uint16_t*)ganZImage.data, (uint16_t*)tempSilhouette.data, ganZImage.cols, objDet, nCenX, nCenY, nDepthThr, index, nCenDepthHist);
	}
	
	//ThresholdDepthImageOnROI((uint16_t*)ganZImage.data, (uint16_t*)tempSilhouette.data, ganZImage.rows, ganZImage.cols, objDet,index, nDepthThr, nCenDepth);
	//cv::dilate(subImg, subImg, getStructuringElement(MORPH_RECT, Size(3, 3)), cv::Point(-1, -1), 1);
	//regionGrow((uint16_t*)ganZImage.data, (uint16_t*)tempSilhouette.data, 
	//	ganZImage.cols, objDet, BodyWidth / 2, BodyLength / 2, nDepthThr, index, nCenDepth);

	return tempSilhouette;
}

DetectedPeopleInfo* PeopleDetector::GetHeadFromSilhouette()
{
	int32_t nBufSzForImage = inpWidth * inpHeight * elementSize;

	Mat ganZImage(inpWidth, inpHeight, CV_16UC1);
	std::memcpy(ganZImage.data, pDepthImage, nBufSzForImage);

	Mat drawing = Mat::zeros(inpWidth, inpHeight, CV_8UC3);
	uint8_t* mask = GetSilhouette();
	Mat capturedSilhouette(inpWidth, inpHeight, CV_8UC1);
	std::memcpy(capturedSilhouette.data, mask, inpWidth * inpHeight);

	DetectedObjectInfo objDet[MAX_DETECTED_OBJECTS];
	int32_t numPeopleDetected = pobjDet->GetDetections(objDet, PERSON_CLASS, PEOPLE_DETECTION_THRESHOLD);
	numPeopleDetected = numPeopleDetected < MAX_PEOPLE_DETECTION ? numPeopleDetected : MAX_PEOPLE_DETECTION;

	for (int i = 0; i < numPeopleDetected; i++)
	{
		int y_start, y_end;
		float ratio1 = (objDet[i].nMaxX - objDet[i].nMinX)
			/ (float)(objDet[i].nMaxY - objDet[i].nMinY);
		float ratio2 = 1 / ratio1;
		if (!bEnableFullBodyTrack && ratio1 <= 0.7)
		{
			y_start = objDet[i].nMinY;
			y_end = ((objDet[i].nMaxY - objDet[i].nMinY) * 6) / 10 + y_start;
		}
		else
		{
			y_start = objDet[i].nMinY;
			y_end = objDet[i].nMaxY;
		}
		objDet[i].nMinY = y_start;
		objDet[i].nMaxY = y_end;

		int16_t BodyWidth = objDet[i].nMaxX - objDet[i].nMinX;
		int16_t BodyLength = objDet[i].nMaxY - objDet[i].nMinY;
		Mat thr = Mat::zeros(BodyLength, BodyWidth, CV_8UC1);
		//Mat thr = Mat::zeros(croppedSilhouette.size(), CV_8UC1);
		uint8_t* DrawSil = (uint8_t*)capturedSilhouette.data;
		uint8_t* DrawThr = (uint8_t*)thr.data;
		for (int nY = objDet[i].nMinY, ty = 0; nY < objDet[i].nMaxY; nY++, ty++)
		{
			for (int nX = objDet[i].nMinX, tx = 0; nX < objDet[i].nMaxX; nX++, tx++)
			{
				if (DrawSil[nY * capturedSilhouette.cols + nX] > 0)
					DrawThr[ty * thr.cols + tx] = 255;
				else
					DrawThr[ty * thr.cols + tx] = 0;
			}
		}

		int nBlockSize = BLOCK_SIZE;
		//Connected Labelling
		cv::Point centroid = GetCentroidOfMaxObjectOnROI(thr, BodyLength, BodyWidth, &objDet[i]);
		int nCenDepth = CalculateDepth((uint16_t*)ganZImage.data, ganZImage.cols, ganZImage.rows, objDet[i].nMinX + centroid.x, objDet[i].nMinY + centroid.y, nBlockSize, nBlockSize);

		// There is a linear relationship with xWidth of head and depth.
		// at depth 1 m, head width ~ 200
		// at depth 2 m, head width ~ 100
		// at depth 3 m, head width ~ 65 to 70
		// at depth 4 m, head width ~ 50
		// depending upon depth the size of bounding box is computed. x_const is 200.
		int x_const = DEPTH_CONSTANT;
		nCenDepth /= 1000; //to convert depth into meters
		int headWidth = (nCenDepth != 0) ? x_const / nCenDepth : 0;
		Mat drawing = Mat::ones(thr.size(), CV_8UC1);
		drawing *= 255;
		cv::Point min_xy = GetMaximumHeadPointOnSilloheutteOnROI(thr, &objDet[i], headWidth, drawing);

		int centre_x = objDet[i].nMinX + min_xy.x;
		int centre_y = objDet[i].nMinY + min_xy.y + headWidth / 2 + headWidth / 10;
		//center_y is moved headwidth/10 spaces lower to neglect the hair portion of head

		poHeadDetections[i].TopLeft.x = (centre_x - headWidth / 2 > objDet[i].nMinX) ? centre_x - headWidth / 2 : objDet[i].nMinX;
		poHeadDetections[i].TopLeft.y = centre_y - headWidth / 2;
		poHeadDetections[i].BottomRight.x = (centre_x + headWidth / 2 < objDet[i].nMaxX) ? centre_x + headWidth / 2 : objDet[i].nMaxX;
		poHeadDetections[i].BottomRight.y = centre_y + headWidth / 2;
		poHeadDetections[i].nBody = UPPER_BODY;
		poHeadDetections[i].nLabelId = i;

		//When there is no contours and so no head detection the bounding boxes
		if (min_xy.x == 0 && min_xy.y == 0) {
			poHeadDetections[i].TopLeft.x = 0;
			poHeadDetections[i].TopLeft.y = 0;
			poHeadDetections[i].BottomRight.x = 0;
			poHeadDetections[i].BottomRight.y = 0;
		}
	}
	return poHeadDetections;
}

DetectedObjectInfo PeopleDetector::GetHeadFromRoi(DetectedObjectInfo objDet)
{
	DetectedObjectInfo pPredictedHead;
	int32_t nBufSzForImage = inpWidth * inpHeight * elementSize;

	Mat ganZImage(inpWidth, inpHeight, CV_16UC1);
	std::memcpy(ganZImage.data, pDepthImage, nBufSzForImage);

	Mat drawing = Mat::zeros(inpWidth, inpHeight, CV_8UC3);
	uint8_t* mask = GetSilhouette();
	Mat capturedSilhouette(inpWidth, inpHeight, CV_8UC1);
	std::memcpy(capturedSilhouette.data, mask, inpWidth * inpHeight);

	int y_start, y_end;
	float ratio1 = (objDet.nMaxX - objDet.nMinX)
		/ (float)(objDet.nMaxY - objDet.nMinY);
	float ratio2 = 1 / ratio1;
	if (!bEnableFullBodyTrack && ratio1 <= 0.7)
	{
		y_start = objDet.nMinY;
		y_end = ((objDet.nMaxY - objDet.nMinY) * 6) / 10 + y_start;
	}
	else
	{
		y_start = objDet.nMinY;
		y_end = objDet.nMaxY;
	}
	objDet.nMinY = y_start;
	objDet.nMaxY = y_end;

	int16_t BodyWidth = objDet.nMaxX - objDet.nMinX;
	int16_t BodyLength = objDet.nMaxY - objDet.nMinY;
	Mat thr = Mat::zeros(BodyLength, BodyWidth, CV_8UC1);
	uint8_t* DrawSil = (uint8_t*)capturedSilhouette.data;
	uint8_t* DrawThr = (uint8_t*)thr.data;
	for (int nY = objDet.nMinY, ty = 0; nY < objDet.nMaxY; nY++, ty++)
	{
		for (int nX = objDet.nMinX, tx = 0; nX < objDet.nMaxX; nX++, tx++)
		{
			if (DrawSil[nY * capturedSilhouette.cols + nX] > 0)
				DrawThr[ty * thr.cols + tx] = 255;
			else
				DrawThr[ty * thr.cols + tx] = 0;
		}
	}

	int nBlockSize = BLOCK_SIZE;
	//Connected Labelling
	cv::Point centroid = GetCentroidOfMaxObjectOnROI(thr, BodyLength, BodyWidth, &objDet);
	int nCenDepth = CalculateDepth((uint16_t*)ganZImage.data, ganZImage.cols, 
		ganZImage.rows, objDet.nMinX + centroid.x, objDet.nMinY + centroid.y,
		nBlockSize, nBlockSize);

	// There is a linear relationship with xWidth of head and depth.
	// at depth 1 m, head width ~ 200
	// at depth 2 m, head width ~ 100
	// at depth 3 m, head width ~ 65 to 70
	// at depth 4 m, head width ~ 50
	// depending upon depth the size of bounding box is computed. x_const is 200.
	int x_const = DEPTH_CONSTANT;
	nCenDepth /= 1000; //to convert depth into meters
	int headWidth = (nCenDepth != 0) ? x_const / nCenDepth : 0;
	Mat drawing2 = Mat::ones(thr.size(), CV_8UC1);
	drawing2 *= 255;
	cv::Point min_xy = GetMaximumHeadPointOnSilloheutteOnROI(thr, &objDet, headWidth, drawing2);

	int centre_x = objDet.nMinX + min_xy.x;
	int centre_y = objDet.nMinY + min_xy.y + headWidth / 2 + headWidth / 10;
	//center_y is moved headwidth/10 spaces lower to neglect the hair portion of head

	//When there is no contours and so no head detection the bounding boxes
	if (min_xy.x == 0 && min_xy.y == 0) {
		pPredictedHead.nMinX = 0;
		pPredictedHead.nMinY = 0;
		pPredictedHead.nMaxX = 0;
		pPredictedHead.nMaxY = 0;
		pPredictedHead.nClass = HEAD_CLASS;
		pPredictedHead.nConfidence = 0;
		pPredictedHead.nId = 0;

	}
	else
	{
		pPredictedHead.nMinX = (centre_x - headWidth / 2 > objDet.nMinX) ?
			centre_x - headWidth / 2 : objDet.nMinX;
		pPredictedHead.nMinY = centre_y - headWidth / 2;
		pPredictedHead.nMaxX = (centre_x + headWidth / 2 < objDet.nMaxX) ?
			centre_x + headWidth / 2 : objDet.nMaxX;
		pPredictedHead.nMaxY = centre_y + headWidth / 2;
		pPredictedHead.nClass = HEAD_CLASS;
		pPredictedHead.nConfidence = 100;
		pPredictedHead.nId = 0;
	}
	
	return pPredictedHead;
}

void PeopleDetector::CloseModule()
{
	//nothing to do here for now
}
