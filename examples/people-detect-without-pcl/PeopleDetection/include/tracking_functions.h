/* 
******************************************************************************
Copyright (c), 2008-2015 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
/*!
 * @file     tracking_functions.h
 *
 * @brief    Internal APIs for tracking
 *
 * @details  This file contains APIs, globals used internally for tracking 
 *
 */

#ifndef __TRACKING_FUNCTIONS_H__
#define __TRACKING_FUNCTIONS_H__

/*=============  I N C L U D E S   =============*/
#include <fstream>
#include <iostream>
#include "object_track.h"

/*==============  D E F I N E S  ===============*/

extern tTrackNode*     gpTrackNodeHead;
extern tToBeTrackNode* gpToBeTrackNodeHead;
extern int             gGlobalLabel;
extern int             gFrameRate;
extern int             gMaxNumTrkNodes;
extern int             gMaxNumObjs;
extern int             gMaxTrajectoryEntries;
extern int             gnErrorCode;

extern
int ObjectTrackingList(
    ObjFeat* ptLabel,
    int numLabels,
    ADI_TRACK_OBJ_STATUS* ptTrackObjectStatus
);

extern
tNode* InitFreeList(
    tNode** pListHead,
    unsigned char* pMemPool,
    int            nNodes,
    int            nSize
);
#endif /*__TRACKING_FUNCTIONS_H__*/

