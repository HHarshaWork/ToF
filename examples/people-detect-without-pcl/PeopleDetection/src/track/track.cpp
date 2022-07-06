/*
******************************************************************************
Copyright (c), 2008-2015 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
/*!
 * @file     track.c
 *
 * @brief    API class for tracking
 *
 * @details  This file contains API functions for initialization of tracking instance and tracking processing 
 *
 */

/*=============  I N C L U D E S   =============*/
#include <stdlib.h>
#include <string.h>
#include "track.h"
#include "object_track.h"
/*=============  D E F I N E S  =============*/

/*=============  E X T E R N A L S  ============*/

/*=============  C O D E  =============*/
int frame_num;

Tracker::Tracker(int nMaxObjects, int nMaxTimeTrackTrajectory)
{
	pPrivInstance = new TrackerPrivate(nMaxObjects, nMaxTimeTrackTrajectory);
	nErrorCode = ADI_TRACK_ERROR_NONE;
	nNumObjects = 0;
	nNumTrackedObjects = 0;
}
Tracker::~Tracker()
{
	delete (TrackerPrivate*)pPrivInstance;
}

bool Tracker::IsInitialized()
{
	return ((TrackerPrivate*)pPrivInstance)->IsInit();
}

void Tracker::Process(ObjFeat* objs, int numObjs, int frameNum)
{
	frame_num = frameNum;
	nNumObjects = numObjs;
	((TrackerPrivate*)pPrivInstance)->Process(objs, numObjs);
}

void Tracker::SetDepthImage(uint16_t *pImage, int16_t* pPointCloudImage)
{
	if (pImage != NULL)
	{
		((TrackerPrivate*)pPrivInstance)->SetDepthImage(pImage, pPointCloudImage);
	}	
}

void Tracker::GetTrackedObjects(TrackedObj** trkObjs, int& numTrkObjs)
{
	((TrackerPrivate*)pPrivInstance)->GetTrackedObjects(trkObjs, numTrkObjs);
	nNumTrackedObjects = numTrkObjs;
}

void Tracker::SetFramerate(int framerate)
{
	((TrackerPrivate*)pPrivInstance)->SetFramerate(framerate);
}

void Tracker::SetThresholdMultFactor(int nFact)
{
	((TrackerPrivate*)pPrivInstance)->SetThresholdMultFactor(nFact);
}

void Tracker::Enable3DTracking(bool bEnable)
{
	((TrackerPrivate*)pPrivInstance)->Enable3DTracking(bEnable);
}

void Tracker::SetDepthImageDim(uint16_t nWidth, uint16_t nHeight)
{
	((TrackerPrivate*)pPrivInstance)->SetDepthImageDim(nWidth, nHeight);
}