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
#include "object_track.h"
#include "tracking_functions.h"
/*=============  D E F I N E S  =============*/

/*=============  E X T E R N A L S  ============*/

/*=============  C O D E  =============*/

TrackerPrivate::TrackerPrivate(int nMaxObjects, int nMaxTimeTrackTrajectory)
{
    nFrameRate = 10;
    nMinRectSize = 600;         
    nWidth = 1024;                        
    nHeight = 1024;                       
    ptTrackedObjects = new TrackedObj[nMaxObjects];      /* Tracked objects returned by tracking */
    ptTrackObjectStatus = new ADI_TRACK_OBJ_STATUS;  /* Internal Tracking Lists */
    nNumObjects = 0;          /* Number of Objects in the current frame which are to be tracked */
    nNumTrackedObjects = 0;   /* Number of Objects which could be tracked in the current frame */
    nConfigMaxTrackNodes = MAX_TRACK_NODES;
    nConfigMaxToBeTrackedNodes = MAX_TO_BE_TRACK_NODES;
    nConfigMaxObjs = nMaxObjects;

    ptTrackObjectStatus->ptFreeTrackListHead = NULL;
    ptTrackObjectStatus->ptFreeTrackListTail = NULL;
    ptTrackObjectStatus->ptFreeToBeListHead = NULL;
    ptTrackObjectStatus->ptFreeToBeListTail = NULL;
    ptTrackObjectStatus->ptTrackListHead = NULL;
    ptTrackObjectStatus->ptTrackListTail = NULL;
    ptTrackObjectStatus->ptToBeTrackListHead = NULL;
    ptTrackObjectStatus->ptToBeTrackListTail = NULL;

    int nSizeOfMemPool = nConfigMaxTrackNodes * sizeof(tTrackNode);
    unsigned char *ptMemory = new unsigned char[nSizeOfMemPool];
    ptMemTrack = ptMemory;
    ptTrackObjectStatus->ptFreeTrackListTail = (tTrackNode*)InitFreeList((tNode**)&ptTrackObjectStatus->ptFreeTrackListHead,
        ptMemory,
        nConfigMaxTrackNodes,
        sizeof(tTrackNode)
    );

    nSizeOfMemPool = nConfigMaxToBeTrackedNodes * sizeof(tToBeTrackNode);
    ptMemory = new unsigned char[nSizeOfMemPool];
    ptMemToBeTrack = ptMemory;
    ptTrackObjectStatus->ptFreeToBeListTail = (tToBeTrackNode*)InitFreeList((tNode**)&ptTrackObjectStatus->ptFreeToBeListHead,
        ptMemory,
        nConfigMaxToBeTrackedNodes,
        sizeof(tToBeTrackNode));

    nSizeOfMemPool = nConfigMaxTrackNodes * sizeof(tOccNode);
    ptMemory = new unsigned char[nSizeOfMemPool];
    ptMemOcc = ptMemory;
    ptTrackObjectStatus->tOcclusionPool.ptFreeTail = (tOccNode*)InitFreeList((tNode**)&(ptTrackObjectStatus->tOcclusionPool.ptFreeHead),
        ptMemory,
        nConfigMaxTrackNodes,
        sizeof(tOccNode));

    ptTrackObjectStatus->pOvlpIndex = new float[nMaxObjects];
    ptTrackObjectStatus->label_assoc = new int[nMaxObjects];
    ptTrackObjectStatus->label_assoc_new = new int[nMaxObjects];
    
    tTrackNode* ptNode = ptTrackObjectStatus->ptFreeTrackListHead;
    /* Initialize the memory for trajectory */
    ptMemTrkTrajectory = new short[2 * nMaxTimeTrackTrajectory * nConfigMaxTrackNodes];
    pt3DMemTrkTrajectory = new short[3 * nMaxTimeTrackTrajectory * nConfigMaxTrackNodes];

    short* ptr = ptMemTrkTrajectory;
    short *PCptr = pt3DMemTrkTrajectory;
    for (int i = 0; i < nConfigMaxTrackNodes; i++)
    {
        ptNode->centroid[0] = ptr;
        ptr += nMaxTimeTrackTrajectory;
        ptNode->centroid[1] = ptr;
        ptr += nMaxTimeTrackTrajectory;

        ptNode->PointCloudcentroid[0] = PCptr;
        PCptr += nMaxTimeTrackTrajectory;
        ptNode->PointCloudcentroid[1] = PCptr;
        PCptr += nMaxTimeTrackTrajectory;
        ptNode->PointCloudcentroid[2] = PCptr;
        PCptr += nMaxTimeTrackTrajectory;

        ptNode = ptNode->next;
    }

    gGlobalLabel = 0;
    gMaxNumTrkNodes = nConfigMaxTrackNodes;
    gMaxNumObjs = nMaxObjects;
    gFrameRate = nFrameRate;
    gMaxTrajectoryEntries = nMaxTimeTrackTrajectory;

    pDepthImg = NULL;
    pPointCloudImg = NULL;
    nMultFactorThreshold = 1;

    ptObjFeat = NULL;
    nErrorCode = 0;
    nMinObjSize = 0;
    ptTrackObjectStatus->nMinObjSize = 0;

    bTrackIn3D = false;
}

TrackerPrivate::~TrackerPrivate()
{
    delete[] ptTrackObjectStatus->pOvlpIndex;
    delete[] ptTrackObjectStatus->label_assoc;
    delete[] ptTrackObjectStatus->label_assoc_new;
    delete[] ptMemTrack;
    delete[] ptMemToBeTrack;
    delete[] ptMemOcc;
    delete[] ptMemTrkTrajectory;
    delete[] pt3DMemTrkTrajectory;
    delete[] ptTrackedObjects;
    delete ptTrackObjectStatus;
}

bool TrackerPrivate::IsInit()
{
    bool bIsInitialized = true;
    if (ptTrackObjectStatus->pOvlpIndex == NULL)
    {
        bIsInitialized = false;
    }
    if (ptTrackObjectStatus->label_assoc == NULL)
    {
        bIsInitialized = false;
    }
    if (ptTrackObjectStatus->label_assoc_new == NULL)
    {
        bIsInitialized = false;
    }
    if (ptMemTrack == NULL)
    {
        bIsInitialized = false;
    }
    if (ptMemToBeTrack == NULL)
    {
        bIsInitialized = false;
    }
    if (ptMemOcc == NULL)
    {
        bIsInitialized = false;
    }
    if (ptMemTrkTrajectory == NULL)
    {
        bIsInitialized = false;
    }
    if (pt3DMemTrkTrajectory == NULL)
    {
        bIsInitialized = false;
    }
    if (ptTrackedObjects == NULL)
    {
        bIsInitialized = false;
    }
    if (pt3DMemTrkTrajectory == NULL)
    {
        bIsInitialized = false;
    }
    if (ptTrackObjectStatus->ptFreeTrackListHead == NULL)
    {
        bIsInitialized = false;
    }
    if (ptTrackObjectStatus->ptFreeToBeListHead == NULL)
    {
        bIsInitialized = false;
    }
    return(bIsInitialized);
}
void TrackerPrivate::Process(ObjFeat* objs, int numObjs)
{
    gpTrackNodeHead = ptTrackObjectStatus->ptTrackListHead;
    gpToBeTrackNodeHead = ptTrackObjectStatus->ptToBeTrackListHead;

    ObjectTrackingList(objs, numObjs, ptTrackObjectStatus);

#if 0
    //check no of free nodes during stability testing
    tTrackNode* FreeTrackNodeHead = ptTrackObjectStatus->ptFreeTrackListHead;
    tToBeTrackNode* FreeToBeTrackNodeHead = ptTrackObjectStatus->ptFreeToBeListHead;
    int nFreeTrackNodes = 0;
    while (FreeTrackNodeHead != NULL)
    {
        nFreeTrackNodes += 1;
        FreeTrackNodeHead = FreeTrackNodeHead->next;
    }

    int nFreeToBeTrackNodes = 0;
    while (FreeToBeTrackNodeHead != NULL)
    {
        nFreeToBeTrackNodes += 1;
        FreeToBeTrackNodeHead = FreeToBeTrackNodeHead->next;
    }

    printf("Free Nodes = %d, Free To Be track Nodes = %d\n", nFreeTrackNodes, nFreeToBeTrackNodes);
#endif
    nErrorCode = gnErrorCode;
}

void TrackerPrivate::GetTrackedObjects(TrackedObj** objs, int& numTrkObjs)
{
    tTrackNode* pTrackNode = ptTrackObjectStatus->ptTrackListHead;
    int i = 0;
    numTrkObjs = 0;
    while (pTrackNode != NULL)
    {
        if (pTrackNode->recency <= 10 && pTrackNode->state != DISAPPEAR)
        //if (pTrackNode->recency <= 10)
        {
            ptTrackedObjects[i].min_x = pTrackNode->min_x;
            ptTrackedObjects[i].min_y = pTrackNode->min_y;
            ptTrackedObjects[i].max_x = pTrackNode->max_x;
            ptTrackedObjects[i].max_y = pTrackNode->max_y;
            ptTrackedObjects[i].centroid_x = pTrackNode->centroid_x;
            ptTrackedObjects[i].centroid_y = pTrackNode->centroid_y;
            ptTrackedObjects[i].area = pTrackNode->area;
            ptTrackedObjects[i].track_label = pTrackNode->label;
            ptTrackedObjects[i].state = 1;
            ptTrackedObjects[i].trkTrajectoryX = &(pTrackNode->centroid[0][0]);
            ptTrackedObjects[i].trkTrajectoryY = &(pTrackNode->centroid[1][0]);
            ptTrackedObjects[i].trkTrajectoryIndx = pTrackNode->index;
            ptTrackedObjects[i].PointCloudTrajectoryX = &(pTrackNode->PointCloudcentroid[0][0]);
            ptTrackedObjects[i].PointCloudTrajectoryY = &(pTrackNode->PointCloudcentroid[1][0]);
            ptTrackedObjects[i].PointCloudTrajectoryZ = &(pTrackNode->PointCloudcentroid[2][0]);
            ptTrackedObjects[i].PointCloudTrajectoryIndx = pTrackNode->PointCloudindex;
            ptTrackedObjects[i].total_count = pTrackNode->total_count;
            ptTrackedObjects[i].track_count = pTrackNode->match_count;
            ptTrackedObjects[i].bPredicted = pTrackNode->bPredicted;

            i++;
            if (i >= nConfigMaxObjs) break;
        }
        pTrackNode = pTrackNode->next;
    }
    *objs = ptTrackedObjects;
    numTrkObjs = i;
}

void TrackerPrivate::SetFramerate(int framerate)
{
    nFrameRate = framerate;
    gFrameRate = nFrameRate;
}