/* 
******************************************************************************
Copyright (c), 2008-2015 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
/*!
 * @file     object_tracking.c
 *
 * @brief    Performs Object Tracking
 *
 * @details  This file implements the functions for Object Tracking. It tracks objects
 *           in current frame to already existing tracked objects or objects in to be tracked
 *           lists. If an object is continuously tracked over the last M frames (10 say), then it
 *           is moved from the to be tracked list to the tracked list, lest it is deleted from the 
 *           to be tracked list. If an object is matched to either of the lists, then its track id 
 *           is updated. If an object cannot be matched then it is added to the To Be track list 
 *           as a new entry. An unmatched object may also be matched with the occlusion objects
 *           to find if it had disappeared and again re-appeared. Finally, the tracked list and the
 *           to be tracked list are scanned to find any objects whose recency count (it has not been
 *           matched in the recent N frames) is high, and are deleted from the lists.
 */

/*=============  I N C L U D E S   =============*/
#include <stdlib.h>
#include "object_track.h"

/*=============  D E F I N E S  =============*/
//#define DEBUG

#define ABS(x) ((x) >= 0 ? (x) : (-(x)) )

#define DISTANCE(x1,x2,y1,y2) (((x1-x2)*(x1-x2)) + ((y1-y2)*(y1-y2)))
#define DISTANCE_3D(x1,x2,y1,y2,z1,z2) (((x1-x2)*(x1-x2)) + ((y1-y2)*(y1-y2)) + ((z1-z2)*(z1-z2)))

#define RECENCY_TO_BE_TRACKED

/* Tracking related thresholds */
#define TRACK_FACTOR 0.2
#define TRACK_RECENCY 5

#define INVALID_PIXEL  0

/*=============  E X T E R N A L S  ============*/

/*=============  D A T A  =============*/


tTrackNode     *gpTrackNodeHead;
tToBeTrackNode *gpToBeTrackNodeHead;
int             gGlobalLabel;
int             gFrameRate;
int             gMaxNumTrkNodes;
int             gMaxNumObjs;
int             gMaxTrajectoryEntries;
int             gnErrorCode;

#if 0

float           gOvlpIndex[ADI_MAX_TRACK_NODES][ADI_MAX_NUM_OBJECTS];
#endif

void debug_output_tracklist(
                            tToBeTrackNode   *ptToBeTrackNode,
                            tTrackNode       *ptTrackNode,
                            TrackedObj *ptLabel,
                            int              numLabels
                           );

/*=============  C O D E  =============*/

extern
int        DecideTrackSplitBeforeMerge(
                                       tNode            *ptNodeHead,
                                       ObjFeat          *ptLabel,
                                       int              numLabels,
                                       int              *label_assoc,
                                       float            *aOvlpIndex,
                                       int16_t         *p3dImage,
                                       uint16_t         nImageWidth,
                                       uint16_t         nImageHeight
                                      );


extern
int         DecideTrackMerge(
                             tNode            *ptNodeHead,
                             ObjFeat          *ptLabel,
                             tOccPool         *ptOccPool,
                             int              numLabels,
                             int              *label_assoc,
                             float            *aOvlpIndex,
                             int16_t         *p3DImage,
                             uint16_t         nImageWidth,
                             uint16_t         nImageHeight);

/*********************** LINK LIST FUNCTIONS ***********************/
/**
 * @brief Allocate node from free list
 *
 * @details This function returns a node from the free pool and updates 
 *          the the free list
 *
 * @param [in] tFreeListHead        Pointer to head of free list
 * @param [in] tFreeListTail        Pointer to tail of free list
 *
 * @return
 * Address of the node
 */
tNode* AllocNode(
                 tNode **tFreeListHead,
                 tNode **tFreeListTail
                )
{
    tNode *pNode;

    pNode = *tFreeListHead;
    if(pNode != NULL)
    {
        if(*tFreeListHead == *tFreeListTail)
        {
            if(pNode->next != NULL)
            {
#ifdef TRACK_DEBUG
                printf("Error condition in Alloc Node\n");
#else
                gnErrorCode = ADI_TRACK_ERROR_ALLOC_NODE;
#endif
            }
            *tFreeListTail = pNode->next;
        }
        *tFreeListHead = pNode->next;
        pNode->next    = NULL;
    }
    return pNode;
}

/**
 * @brief Insert node
 *
 * @details This function inserts a node into a list and updates it
 *
 * @param [in] tFreeListHead        Pointer to head of list
 * @param [in] tFreeListTail        Pointer to tail of list
 * @param [in] pNode                Node to be inserted
 *
 * @return
 * None
 */
void InsertNode(
                tNode **tFreeListHead,
                tNode **tFreeListTail, 
                tNode *pNode
               )
{
    pNode->next = NULL;
    if(*tFreeListTail != NULL)
    {
        (*tFreeListTail)->next = pNode;
        *tFreeListTail       = pNode;
    }
    else
    {
        *tFreeListTail = pNode;
        *tFreeListHead = pNode;
    }
    return;
}

/**
 * @brief Remove node
 *
 * @details This function removes a node from list and updates the list
 *
 * @param [in] ptListHead        Pointer to head of list
 * @param [in] ptListTail        Pointer to tail of list
 * @param [in] ptNode            Node to be removed
 * @param [in] ptNodePrev        Node previous to the removed node
 *
 * @return
 * None
 */
void RemoveNode(
                tNode **ptListHead,
                tNode **ptListTail, 
                tNode *ptNode,
                tNode *ptNodePrev
               )
{
    tNode *ptNodeNext = ptNode->next;

    if(ptNode == *ptListHead) 
    {
        /* first node in the list */
        *ptListHead = ptNodeNext;
    }
    else
    {
        /* middle node in the list */
        if (ptNodePrev != NULL)
        {
            ptNodePrev->next = ptNodeNext;
        }
    }
           
    if(ptNode == *ptListTail)
    {
        /* last node in the list */
        *ptListTail = ptNodePrev;
    }
}

/**
* @brief Calculate average depth around centre
*
* @details This function calculates the average 3D depth around a window of the centre pixel
*
* @param [in]     p3DDepthImage   Image containing depth value at pixels
* @param [in]     nImageWidth   width of image
* @param [in]     nImageHeight  height of image
* @param [in]     nCentreX      centre point X of box to calculate depth
* @param [in]     nCentreY      centre point Y of box to calculate depth
* @param [out]     pOutX        3D output of X location
* @param [out]     pOutY        3D output of Y location
* @param [out]     pOutZ        3D output of Z location
*
* @return
* None
*/
void Get3DPointCloud(
    int16_t* p3DDepthImage,
    uint16_t nImageWidth,
    uint16_t nImageHeight,
    uint16_t  nCentreX,
    uint16_t  nCentreY,
    int16_t* pOutX,
    int16_t* pOutY,
    int16_t* pOutZ
)
{
    uint16_t nAverageDepth;
    int16_t nAverageX = 0;
    int16_t nAverageY = 0;
    int32_t nSumX = 0;
    int32_t nSumY = 0;
    int32_t nWindowSum = 0;

    int16_t* pIn;

    //checking depth on a small depth around centre pixel
    int nWindowSize = POINTCLOUD_WINDOW_SIZE;

    //make window size multiple of 2
    int nValidPixels = 0;
    for (int nRow = MAX(0, nCentreY - nWindowSize / 2); nRow < MIN(nImageHeight - 1, nCentreY + nWindowSize / 2); nRow++)
    {
        pIn = &p3DDepthImage[nImageWidth * nRow * 3 + (nCentreX - nWindowSize / 2) * 3];
        for (int nCol = 0; nCol < nWindowSize; nCol++)
        {
            if (*pIn != INVALID_PIXEL)
            {
                nValidPixels += 1;
                nSumX += *pIn;
                nSumY += *(pIn + 1);
                nWindowSum += *(pIn + 2);
            }
            pIn += 3;
        }
    }

    //max depth value is 1024 so we have enough head room with a 32 bit sum
    if (nValidPixels != 0)
    {
        nAverageDepth = (int16_t)(nWindowSum / (nValidPixels));
        nAverageX = (int16_t)(nSumX / (nValidPixels));
        nAverageY = (int16_t)(nSumY / (nValidPixels));
    }
    else
    {
        nAverageDepth = 0;
        nAverageX = 0;
        nAverageY = 0;
    }

    *pOutX = nAverageX;
    *pOutY = nAverageY;
    *pOutZ = nAverageDepth;
}

/**
 * @brief Update node properties
 *
 * @details This function updates the properties of the given node
 *
 * @param [in,out]  pNode                Node to be updated
 * @param [in]      ptLabelCurrentFrame  Properties of node
 *
 * @return
 * Always returns 0
 */
int UpdateNode(
               tNode            *pNode,
               ObjFeat *ptLabelCurrentFrame,
               uint16_t* pDepthImage,
               int16_t*  p3dImage,
               uint16_t nImageWidth,
               uint16_t nImageHeight
              )
{
    int i;
    if(pNode->objectlabel_id == -1)
    {
        /* No match. No update */
        pNode->recency     = pNode->recency + 1;
        pNode->total_count = pNode->total_count + 1;
    }
    else
    {
        /* Match found */

        /* If detected node, copy coordinates; if predicted use sad*/
        i = pNode->objectlabel_id;
        if (ptLabelCurrentFrame[i].detection_type == DETECTED)
        {
            pNode->recency = 0;
            pNode->total_count = pNode->total_count + 1;
            pNode->match_count = pNode->match_count + 1;
            pNode->min_x = ptLabelCurrentFrame[i].min_x;
            pNode->min_y = ptLabelCurrentFrame[i].min_y;
            pNode->max_x = ptLabelCurrentFrame[i].max_x;
            pNode->max_y = ptLabelCurrentFrame[i].max_y;
            pNode->area = ptLabelCurrentFrame[i].area;
            pNode->centroid_x = ptLabelCurrentFrame[i].centroid_x;
            pNode->centroid_y = ptLabelCurrentFrame[i].centroid_y;
            pNode->depth_val = CalculateDepth(pDepthImage, nImageWidth, nImageHeight,
                (pNode->min_x + pNode->max_x) >> 1,
                (pNode->min_y + pNode->max_y) >> 1,
                (pNode->max_x - pNode->min_x),
                (pNode->max_y - pNode->min_y));//average depth of centre window pixels
            pNode->bPredicted = false;

            uint16_t nCenX = (pNode->min_x + pNode->max_x) >> 1;
            uint16_t nCenY = (pNode->min_y + pNode->max_y) >> 1;
            int16_t n3dX, n3dY, n3dZ;
            Get3DPointCloud(p3dImage, nImageWidth, nImageHeight,
                nCenX, nCenY, &n3dX, &n3dY, &n3dZ);

            //Add 3d location
            pNode->PointCloud_x = n3dX;
            pNode->PointCloud_y = n3dY;
            pNode->PointCloud_z = n3dZ;

            ptLabelCurrentFrame[i].ptr = (void*)pNode;
        }
        else
        {
            //use sad in depth image to find this

#if 0
            pNode->objectlabel_id = -1;
            pNode->recency = pNode->recency + 1;
            pNode->total_count = pNode->total_count + 1;
#else

            //approximate location of head, search in window around this
            uint16_t nCentreX = (ptLabelCurrentFrame[i].max_x + ptLabelCurrentFrame[i].min_x) >> 1;
            uint16_t nCentreY = (ptLabelCurrentFrame[i].max_y + ptLabelCurrentFrame[i].min_y) >> 1;

            //use this window dimensions
            uint16_t nBoxWidth = (ptLabelCurrentFrame[i].max_x - ptLabelCurrentFrame[i].min_x);
            uint16_t nBoxHeight = (ptLabelCurrentFrame[i].max_y - ptLabelCurrentFrame[i].min_y);

            /*
            * Since we are doing this using, silhouette no need to find better position
            bool bHeadLocationFound = FindHeadUsingDepth(pDepthImage, 
                                                         nImageWidth, 
                                                         nImageHeight,
                                                         &nCentreX, 
                                                         &nCentreY, 
                                                         nBoxWidth, 
                                                         nBoxHeight, 
                                                         pNode->depth_val,
                                                         DEPTH_COMPARISON_THRESHOLD,
                                                         SEARCH_WINDOW_PIXELS,
                                                         SEARCH_STRIDE);
                                                         */
            //Design Note
            //Earlier we were getting a probable head location and using 
            //depth to find the best match. Now we are getting the head
            //location from silhouette which already reuses a lot of this
            //analysis. Hence we are using the silhouette predicted head
            //directly without any processing. 
            /*
            bool bHeadLocationFound = true;
            if (bHeadLocationFound == true)
            */
            {
                //predicted bounding box size from matched object
                pNode->recency = 0;// pNode->recency + 1;//not updating recency and match count due to this being predicted
                pNode->match_count = pNode->match_count + 1;

                pNode->total_count = pNode->total_count + 1;  
                int alpha = 0;
                int min_x = nCentreX - nBoxWidth / 2;
                min_x = alpha * pNode->min_x / 10 + (10 - alpha) * min_x / 10;
                int min_y = nCentreY - nBoxHeight / 2;
                min_y = alpha * pNode->min_y / 10 + (10 - alpha) * min_y / 10;
                int max_x = nCentreX + nBoxWidth / 2;
                max_x = alpha * pNode->max_x / 10 + (10 - alpha) * max_x / 10;
                int max_y = nCentreY + nBoxHeight / 2;
                max_y = alpha * pNode->max_y / 10 + (10 - alpha) * max_y / 10;

                pNode->min_x = min_x;
                pNode->max_x = max_x;
                pNode->min_y = min_y;
                pNode->max_y = max_y;
                pNode->area = (pNode->max_x - pNode->min_x) *
                    (pNode->max_y - pNode->min_y);
                pNode->centroid_x = (pNode->max_x + pNode->min_x) / 2 * pNode->area;
                pNode->centroid_y = (pNode->max_y + pNode->min_y) / 2 * pNode->area;
                pNode->depth_val = CalculateDepth(pDepthImage, nImageWidth, nImageHeight,
                    (pNode->min_x + pNode->max_x) >> 1,
                    (pNode->min_y + pNode->max_y) >> 1,
                    (pNode->max_x - pNode->min_x),
                    (pNode->max_y - pNode->min_y));//average depth of centre window pixels
                pNode->bPredicted = true;
                ptLabelCurrentFrame[i].ptr = (void*)pNode;
            }
            /*
            else
            {
                //this node is not matched since depth is out of threshold
                pNode->objectlabel_id = -1;
                pNode->recency = pNode->recency + 1;
                pNode->total_count = pNode->total_count + 1;
            }
            */
#endif
        }
    }
    return 0;
}



/**
 * @brief Update specific node properties
 *
 * @details This function updates specific properties of a tracked node only
 *
 * @param [in,out] pNode                Node to be updated
 * @param [in]     ptLabelCurrentFrame  Pointer to structure holding properties of all nodes
 *
 * @return
 * Always returns 0
 */
int UpdateTrackNode(
                    tTrackNode       *pNode,
                    ObjFeat          *ptLabelCurrentFrame,
                    int16_t         *p3DImage,
                    uint16_t         nImageWidth,
                    uint16_t         nImageHeight
                    )
{
    if(pNode->objectlabel_id != -1)
    {
        pNode->index = pNode->index == gMaxTrajectoryEntries - 1 ? 0:(pNode->index + 1);
        pNode->centroid[0][pNode->index] = (short)(pNode->centroid_x/(float)pNode->area);
        pNode->centroid[1][pNode->index] = (short)(pNode->centroid_y/(float)pNode->area);
        //pNode->state = TRACK;
        //Update histogram here?? 
        uint16_t nCenX = (pNode->min_x + pNode->max_x) >> 1;
        uint16_t nCenY = (pNode->min_y + pNode->max_y) >> 1;
        int16_t n3dX, n3dY, n3dZ;
        Get3DPointCloud(p3DImage, nImageWidth, nImageHeight,
            nCenX, nCenY, &n3dX, &n3dY, &n3dZ);

        pNode->PointCloudindex = pNode->PointCloudindex == gMaxTrajectoryEntries - 1 ? 0 : (pNode->PointCloudindex + 1);
        pNode->PointCloudcentroid[0][pNode->PointCloudindex] = n3dX;
        pNode->PointCloudcentroid[1][pNode->PointCloudindex] = n3dY;
        pNode->PointCloudcentroid[2][pNode->PointCloudindex] = n3dZ;
    }
    else
    {
        pNode->state = DISAPPEAR;
    }
    pNode->bSplit = 0;
    return 0;
}

/**
 * @brief Copy features from a node to object
 *
 * @details This function copies specific properties from a node to object
 *
 * @param [in]     ptObject   Pointer to object to be updated
 * @param [in,out] ptNode     Pointer to node
 *
 * @return
 * None
 */
void CopyFromNode(
                  tCompare *ptObject,
                  tNode    *ptNode,
                  int16_t *p3dImage,
                  uint16_t nImageWidth,
                  uint16_t nImageHeight
                 )
{
    ptObject->centroid_x = ptNode->centroid_x;
    ptObject->centroid_y = ptNode->centroid_y;
    ptObject->min_x      = ptNode->min_x;
    ptObject->min_y      = ptNode->min_y;
    ptObject->max_x      = ptNode->max_x;
    ptObject->max_y      = ptNode->max_y;
    ptObject->area       = ptNode->area;
    ptObject->PointCloud_x = ptNode->PointCloud_x;
    ptObject->PointCloud_y = ptNode->PointCloud_y;
    ptObject->PointCloud_z = ptNode->PointCloud_z;
}

/**
 * @brief Calculate average depth around centre
 *
 * @details This function calculates the average 3D depth around a window of the centre pixel
 *
 * @param [in]     pDepthImage   Image containing depth value at pixels
 * @param [in]     nImageWidth   width of image
 * @param [in]     nImageHeight  height of image
 * @param [in]     nCentreX      centre point X of box to calculate depth
 * @param [in]     nCentreY      centre point Y of box to calculate depth
 * @param [in]     WindowSize    Window around which to calculate average depth
 * 
 * @return
 * Average depth
 */
uint16_t CalculateDepth(
    uint16_t* pDepthImage,
    uint16_t nImageWidth,
    uint16_t nImageHeight,
    int16_t  nCentreX,
    int16_t  nCentreY,
    int16_t  nWidthX,
    int16_t  nWidthY
)
{
    uint16_t nAverageDepth;
    uint32_t nWindowSum = 0;

    uint16_t* pIn;

    //checking depth on a small depth around centre pixel
    nWidthX = POINTCLOUD_WINDOW_SIZE;
    nWidthY = POINTCLOUD_WINDOW_SIZE;

    //make window size multiple of 2
    int nValidPixels = 0;
    for (int nRow = MAX(0,nCentreY - nWidthY /2); nRow < MIN(nImageHeight - 1, nCentreY + nWidthY /2); nRow++)
    {
        pIn = &pDepthImage[nImageWidth * nRow + (nCentreX - nWidthX / 2)];
        for (int nCol = 0; nCol < nWidthX; nCol++)
        {
            if (*pIn != INVALID_PIXEL)
            {
                nValidPixels += 1;
                nWindowSum += *pIn;
            }
            pIn++;
        }
    }

    //max depth value is 1024 so we have enough head room with a 32 bit sum
    if (nValidPixels != 0)
    {
        nAverageDepth = (uint16_t)(nWindowSum / (nValidPixels));
    }
    else
    {
        nAverageDepth = 0;
    }
    
    return (nAverageDepth);
}

/**
 * @brief Use sum of abosulte differences to find where the head location could lie
 *
 * @details This function calculates the average 3D depth around a window of the centre pixel
 *
 * @param [in]       pDepthImage   Image containing depth value at pixels
 * @param [in]       nImageWidth   width of image
 * @param [in]       ImageHeight  height of image
 * @param [in,out]   nCentreX      centre point X of box to calculate depth
 * @param [in,out]   nCentreY      centre point Y of box to calculate depth
 * @param [in]       nBoxWidth    Width of Window around which to calculate average depth
 * @param [in]       nBoxHeight   Height of Window around which to calculate average depth 
 * @param [in]       RefDepth        Ref depth to compare against
 * @param [in]       DepthThreshold  minimum depth threshold beyond which no match will be found
 * @param [in]       SearchWin  Window in pixels around which to search for potential minima
 * @param [in]       Stride     Window stride in pixels around which to search for potential minima
 *
 * @return
 * Average true if minima found, else false
 */
bool FindHeadUsingDepth(
    uint16_t* pDepthImage,
    uint16_t nImageWidth,
    uint16_t nImageHeight,
    uint16_t *nCentreX,
    uint16_t *nCentreY,
    uint16_t nBoxWidth,
    uint16_t nBoxHeight,
    uint16_t RefDepth,
    uint16_t DepthThreshold,
    uint16_t WindowSize,
    uint16_t WindowStride)
{
    bool bRet = false;
    //create an array 
    int nSearchVal = WindowSize / WindowStride;
    int32_t nSADVal = INT_MAX;
    
    int nBoxCentrePixelX = *nCentreX;
    int nBoxCentrePixelY = *nCentreY;
    
    //checking depth on a small depth around centre pixel
    nBoxWidth = POINTCLOUD_WINDOW_SIZE;
    nBoxHeight = POINTCLOUD_WINDOW_SIZE;

    //reducing search area in y direction to reduce mis detections
    for (int nRowSearch = nSearchVal/4; nRowSearch <= nSearchVal - nSearchVal/4; nRowSearch++)
    {
        int nStartRow = nBoxCentrePixelY - WindowSize / 2 + nRowSearch * WindowStride
            - nBoxHeight / 2;
        //boundary check 
        if (nStartRow < 0)
        {
            continue;
        }
        if (nStartRow >= nImageHeight)
        {
            continue;
        }
        for (int nColSearch = 0; nColSearch <= nSearchVal; nColSearch++)
        {

            int nStartCol = nBoxCentrePixelX - WindowSize/2 + nColSearch * WindowStride
                - nBoxWidth/2;

            //boundary check 
            if (nStartCol < 0)
            {
                continue;
            }
            if (nStartCol >= nImageWidth)
            {
                continue;
            }

            nStartRow = MAX(nStartRow, 0);
            nStartRow = MIN(nStartRow, nImageHeight - 1 - nBoxHeight);

            nStartCol = MAX(nStartCol, 0);
            nStartCol = MIN(nStartCol, nImageWidth - 1 - nBoxWidth);
            
            uint32_t nSum = 0;
            int nValidPixels = 0;
            for (int nRow = 0; nRow < nBoxHeight; nRow++)
            {
                uint16_t *nDepthPixel = 
                    &pDepthImage[(nStartRow + nRow) * nImageWidth + nStartCol];
                for (int nCol = 0; nCol < nBoxWidth; nCol++)
                {
                    if (*nDepthPixel != INVALID_PIXEL)
                    {
                        nValidPixels += 1;
                        nSum += *nDepthPixel;
                    }
                    nDepthPixel++;
                }
            }
            uint32_t nAvg = 0;
            if (nValidPixels != 0)
            {
                nAvg = (uint16_t)(nSum / (nValidPixels));
            }

            if (abs((int)nAvg - (int)RefDepth) < DepthThreshold)
            {
                if (abs((int)nAvg - (int)RefDepth) < nSADVal)
                {
                    nSADVal = abs((int)nAvg - (int)RefDepth);
                    *nCentreY = (nStartRow + nBoxHeight/2);
                    *nCentreX = (nStartCol + nBoxWidth/2);
                }
            }
        }
    }

    if (nSADVal != INT_MAX)
    {
        bRet = true;
    }
    return bRet;
}

/**
 * @brief Find approximate location of head using body depth pixels
 *
 * @details This function calculates the probably location of head based on body centroid depth
 *
 * @param [in]       pDepthImage   Image containing depth value at pixels
 * @param [in]       nImageWidth   width of image
 * @param [in]       ImageHeight  height of image
 * @param [in,out]   nCentreX      centre point X of body to calculate depth
 * @param [in,out]   nCentreY      centre point Y of body to calculate depth
 * @param [in]       nBoxWidth    Width of Window around which to calculate average depth
 * @param [in]       nBoxHeight   Height of Window around which to calculate average depth
 * @param [in]       MinX         Min X of search window
 * @param [in]       MaxX         Max X of search window
 * @param [in]       MinY         Min Y of search window
 * @param [in]       WindowStride     Window stride in pixels around which to search for potential minima
 *
 * @return
 * Average true if minima found, else false
 */
void FindHeadLocationUsingDepth(
    uint16_t* pDepthImage,
    uint16_t nImageWidth,
    uint16_t nImageHeight,
    uint16_t* nCentreX,
    uint16_t* nCentreY,
    uint16_t nBoxWidth,
    uint16_t nBoxHeight,
    uint16_t nMinX,
    uint16_t nMaxX,
    uint16_t nMinY,
    uint16_t WindowStride)
{
    bool bRet = false;
    //create an array 
    int32_t nMinVal = INT_MAX;

    int nBoxCentrePixelX = *nCentreX;
    int nBoxCentrePixelY = *nCentreY;

    //calculate ref depth on body centroid first
    uint16_t RefDepth = CalculateDepth(pDepthImage, nImageWidth, nImageHeight,
        nBoxCentrePixelX, nBoxCentrePixelY, WINDOW_SIZE, WINDOW_SIZE);

    //checking depth on a small depth around centre pixel
    //only look across horizontal direction near top
    for (int nColSearch = nMinX; nColSearch < nMaxX - nBoxWidth; nColSearch += WindowStride)
    {
        int nStartCol = nColSearch;

        uint32_t nSum = 0;
        int nValidPixels = 0;
        for (int nRow = nMinY; nRow < nMinY + nBoxHeight; nRow++)
        {
            uint16_t* nDepthPixel =
                &pDepthImage[nRow * nImageWidth + nStartCol];
            for (int nCol = 0; nCol < nBoxWidth; nCol++)
            {
                if (*nDepthPixel != INVALID_PIXEL)
                {
                    nValidPixels += 1;
                    nSum += *nDepthPixel;
                }
                nDepthPixel++;
            }
        }
        uint32_t nAvg = 0;
        if (nValidPixels != 0)
        {
            nAvg = (uint16_t)(nSum / (nValidPixels));
        }

        if (abs((int)nAvg - (int)RefDepth) < nMinVal)
        {
            nMinVal = abs((int)nAvg - (int)RefDepth);
            *nCentreY = nMinY;
            *nCentreX = (nStartCol + nBoxWidth / 2);
        }
    }
}
/*********************** Algorithm Functions ***************************/

/**
 * @brief Check Tracked Object
 *
 * @details This function determines if the tracked node has to be removed.
 *
 * @param [in] ptNode     Node currently tracked
 * @param [in] framerate  Frame rate
 *
 * @return
 * 0 - If it is has to be removed<br>
 * 1 - If it is to be retained
 */
int IsTrackNodeRemove(
                      tNode *ptNode,
                      int   framerate
                     )
{
    tTrackNode *ptTrackNode = (tTrackNode*)ptNode;

    int return_value = 0;
    if((ptTrackNode->match_count > (TRACK_FACTOR*60*framerate)) && (ptTrackNode->recency > (TRACK_RECENCY*2)))
    {
        return_value = 1;
    }
    //if((ptTrackNode->match_count <= (1*60*framerate)) && (ptTrackNode->recency > (1*60*framerate)))
    if((ptTrackNode->match_count <= (TRACK_FACTOR*60*framerate)) && (ptTrackNode->recency > (TRACK_RECENCY)))
    {
        return_value = 1;
    }
    return return_value; 
}

/**
 * @brief Check To Be Tracked Object
 *
 * @details This function determines if the To Be tracked node has to be removed.
 *
 * @param [in] ptNode     Node currently tracked
 * @param [in] framerate  Frame rate
 *
 * @return
 * 0 - If it is has to be removed<br>
 * 1 - If it is to be retained
 */
int IsToBeNodeRemove(
                     tNode *ptNode,
                     int   framerate
                    )
{
    tToBeTrackNode *ptToBeTrackNode = (tToBeTrackNode*)ptNode;
    int return_value = 0; 

    if(ptToBeTrackNode->recency > 2)
    {
        return_value = 1;
    }
    if(((ptToBeTrackNode->total_count >= 20) && ptToBeTrackNode->match_count <= 10 ))
    {
        return_value = 1;
    }
    return return_value; 
}

/**
 * @brief Compute object properties
 *
 * @details This function computes the centroid and width/height properties of the object
 *
 * @param [in,out] ptObject      Pointer to object to be updated
 *
 * @return
 * None
 */
void CalculateObjectProperties(tCompare *ptObject)
{
    ptObject->centroid_x_actual = (ptObject->centroid_x/(float)ptObject->area);
    ptObject->centroid_y_actual = (ptObject->centroid_y/(float)ptObject->area);
    ptObject->size_x = (ptObject->max_x - ptObject->min_x);
    ptObject->size_y = (ptObject->max_y - ptObject->min_y);
}

/**
 * @brief Compare two objects
 *
 * @details This function compares two objects to see if they are similar
 *          The similarity conditions is based on the following
 *          1. The distance between them is less than a threshold
 *          2. Their aspect ratios match within a threshold
 *          3. They overlap to a greater degree
 *
 * @param [in] ptObject1      Pointer to first object
 * @param [in] ptObject2      Pointer to second object 
 * @param [in] nThreshold     threshold mult factor
 *
 * @return
 * 0 - If they dont merge<br>
 * 1 - If they merge
 */
static
int CompareObjects(
                   tCompare *ptObject1,
                   tCompare *ptObject2,
                   int nThreshold,
                   bool bUse3DTracking
                  )
{
    /* Design note
     3D tracking works well on people where their position is clear
     for head we use prediction sometimes so the computed z value 
     may include a lot of background.
     This skews the matching distance and hence doesnt work well for 
     head location tracking.
     Therefore we are using this only for people and not for head*/

    int return_value = 0;
    if (bUse3DTracking == true)
    {
        float x_var = (float)ptObject1->PointCloud_x - ptObject2->PointCloud_x;
        float y_var = (float)ptObject1->PointCloud_y - ptObject2->PointCloud_y;
        float z_var = (float)ptObject1->PointCloud_z - ptObject2->PointCloud_z;

        float distance = (x_var) * (x_var)+(y_var) * (y_var)+(z_var) * (z_var);

        if (distance < 1000 * 1000)//100mm = 10 cm, 1000mm = 100 cm = 1m = 3feet
        {
            return_value = 1;
        }
    }
    else
    {
        float ratio_object1 = ptObject1->size_x / (float)ptObject1->size_y;
        float ratio_object2 = ptObject2->size_x / (float)ptObject2->size_y;
        float ratio1 = ratio_object1 / ratio_object2;
        float ratio2 = ratio_object2 / ratio_object1;
        float x_var = (float)ptObject1->centroid_x_actual - ptObject2->centroid_x_actual;
        float y_var = (float)ptObject1->centroid_y_actual - ptObject2->centroid_y_actual;

        float distance = (x_var) * (x_var)+(y_var) * (y_var);

        int nOvlpW;
        int nOvlpH;
        int nOvlpArea;
        //Replace with distance condition later
        //if((ABS(ptObject1->centroid_x_actual - ptObject2->centroid_x_actual) < 40) && (ABS(ptObject1->centroid_y_actual - ptObject1->centroid_y_actual) < 40))
        //printf("distance ratio : %f %f %f\n", distance, ratio1, ratio2);

        //rather than absolute distance replace with multiple of object size
        int nObjSize1 = MAX(ptObject1->size_x, ptObject1->size_y);
        int nObjSize2 = MAX(ptObject2->size_x, ptObject2->size_y);
        int nDistThres = MAX(nObjSize1, nObjSize2);//how much can object move in 1 sec
        if (distance < nThreshold * nDistThres * nDistThres)
        {
            //if(((ratio1 >= 0.5) && (ratio1 <= 1)) || ((ratio2 >= 0.5) && (ratio2 <= 1)))
            {
                return_value = 1;
            }
        }
        else
        {
            /* if overlaps by > 80% then also match */
            nOvlpW = (MIN(ptObject1->max_x, ptObject2->max_x)) - (MAX(ptObject1->min_x, ptObject2->min_x));
            nOvlpW = MAX(nOvlpW, 0);

            nOvlpH = (MIN(ptObject1->max_y, ptObject2->max_y) - MAX(ptObject1->min_y, ptObject2->min_y));
            nOvlpH = MAX(nOvlpH, 0);

            nOvlpArea = nOvlpH * nOvlpW;

            //printf("%d %d %d\n", nOvlpArea, ptObject1->size_x * ptObject1->size_y, ptObject2->size_x * ptObject2->size_y);

            if (ptObject1->size_x * ptObject1->size_y < ptObject2->size_x * ptObject2->size_y)
            {
                if ((nOvlpArea >= 0.5 * ptObject1->size_x * ptObject1->size_y)/* &&
                    (((ratio1 >= 0.5) && (ratio1 <= 1)) || ((ratio2 >= 0.5) && (ratio2 <= 1))) */
                    )
                    return_value = 1;
            }
            else
            {
                if ((nOvlpArea >= 0.5 * ptObject2->size_x * ptObject2->size_y)/* &&
                    (((ratio1 >= 0.5) && (ratio1 <= 1)) || ((ratio2 >= 0.5) && (ratio2 <= 1))) */
                    )
                    return_value = 1;
            }

        }
    }
    return (return_value);   
}

/**
 * @brief Resolve better match
 *
 * @details This function resolves one to many match (if two objects are
 *          matched to the same object). The functions finds out which is better
 *          matched based on distance criteria
 *
 * @param [in] ptObject        Pointer to object
 * @param [in] ptObjectMatch1  Pointer to first matched object
 * @param [in] ptObjectMatch2  Pointer to second matched object
 * @param [in] bUse3DTracking  Enable/disable 3d match
 *
 * @return
 * 0 - If first object is better match<br>
 * 1 - If second object is better match
 */
int ConflictResolve(
                    tCompare *ptObject,
                    tCompare *ptObjectMatch1,
                    tCompare *ptObjectMatch2,
                    bool     bUse3DTracking
                   )
{
    float distance1;
    float distance2;
    int return_value = 0;
    if (bUse3DTracking == false)
    {
        distance1 = DISTANCE(ptObject->centroid_x_actual, ptObjectMatch1->centroid_x_actual, ptObject->centroid_y_actual, ptObjectMatch1->centroid_y_actual);
        distance2 = DISTANCE(ptObject->centroid_x_actual, ptObjectMatch2->centroid_x_actual, ptObject->centroid_y_actual, ptObjectMatch2->centroid_y_actual);
    }
    else
    {

        distance1 = (float)DISTANCE_3D(ptObject->PointCloud_x,
            ptObjectMatch1->PointCloud_x,
            ptObject->PointCloud_y,
            ptObjectMatch1->PointCloud_y,
            ptObject->PointCloud_z,
            ptObjectMatch1->PointCloud_z);
        distance2 = (float)DISTANCE_3D(ptObject->PointCloud_x,
            ptObjectMatch2->PointCloud_x,
            ptObject->PointCloud_y,
            ptObjectMatch2->PointCloud_y,
            ptObject->PointCloud_z,
            ptObjectMatch2->PointCloud_z);
    }
    if(distance1 < distance2)
    {
        return_value = 1;
    }
    return (return_value);
}

/**
 * @brief Copy Tracked Object to Object Array
 *
 * @details This function copies tracked object features from list to object array
 *
 * @param [in] ptObject        Pointer to tracked object
 * @param [in] ptNode          Array of objects
 * @param [in] i               Index to the object array
 *
 * @return
 * None
 */
void CopyFromObjFeat(
                     tCompare *ptObject,
                     ObjFeat  *ptNode,
                     int i,
                     int16_t* p3dImage,
                     uint16_t nImageWidth,
                     uint16_t nImageHeight
                    )
{
    ptObject->centroid_x = ptNode[i].centroid_x;
    ptObject->centroid_y = ptNode[i].centroid_y;
    ptObject->min_x      = ptNode[i].min_x;
    ptObject->min_y      = ptNode[i].min_y;
    ptObject->max_x      = ptNode[i].max_x;
    ptObject->max_y      = ptNode[i].max_y;
    ptObject->area       = ptNode[i].area;

    uint16_t nCenX = (ptNode[i].min_x + ptNode[i].max_x) >> 1;
    uint16_t nCenY = (ptNode[i].min_y + ptNode[i].max_y) >> 1;
    int16_t n3dX, n3dY, n3dZ;
    Get3DPointCloud(p3dImage, nImageWidth, nImageHeight,
        nCenX, nCenY, &n3dX, &n3dY, &n3dZ);

    //Add 3d location
    ptObject->PointCloud_x = n3dX;
    ptObject->PointCloud_y = n3dY;
    ptObject->PointCloud_z = n3dZ;
}

/**
 * @brief Update all tracked objects in list
 *
 * @details This function updates the trajectory paths of all tracked objects in list
 *
 * @param [in] pTrackNode      Pointer to Track list
 * @param [in] ptLabel         Objects in current frame
 *
 * @return
 * Always returns 0
 */
static
int UpdateLabelToTrackNode(
                           tTrackNode       *pTrackNode,
                           ObjFeat          *ptLabel,
                           int16_t         *p3DImage,
                           uint16_t         nImageWidth,
                           uint16_t         nImageHeight
                          )
{
    tTrackNode *pNode = pTrackNode;
    
    while(pNode != NULL)
    {
        UpdateTrackNode(pNode, ptLabel, p3DImage, nImageWidth, nImageHeight);
        pNode = pNode->next;
    }
    return 0;
}

/**
 * @brief Update matched objects in track list 
 *
 * @details This function updates the properties of tracked objects which could be matched
 *          to objects in the current frame
 *
 * @param [in] pNodeHead      Pointer to Head of Track List
 * @param [in] ptLabel        Objects in current frame
 *
 * @return
 * Always returns 0
 */
static 
int UpdateMatchedNodes(
                       tNode            *pNodeHead,
                       ObjFeat          *ptLabel,    
                       uint16_t         *pDepthImage,
                       int16_t          *p3DImage,
                       uint16_t         nImageWidth,
                       uint16_t         nImageHeight
                      )
{
    tNode *pNode = pNodeHead;
    
    while(pNode != NULL)
    {
        UpdateNode(pNode, ptLabel, pDepthImage, p3DImage, nImageWidth, nImageHeight);
        pNode = pNode->next;
    }
    return 0;
}

/*****************************Track List manipulation functions *******************/
/**
 * @brief Initialize free list
 *
 * @details This function adds nodes to free list from memory block
 *
 * @param [in,out] pListHead      Pointer to head of list
 * @param [in] pMemPool           Memory from which the nodes will be allocated
 * @param [in] nNodes             Number of nodes to be there in list
 * @param [in] nSize              Size of each node
 *
 *
 * @return
 * List 
 */

tNode* InitFreeList(
                    tNode         **pListHead, 
                    unsigned char *pMemPool,
                    int            nNodes,
                    int            nSize
                   )
{
    int i;
    tNode *pNode;

    pNode       = (tNode*)pMemPool;
    *pListHead  = pNode;
    pMemPool  += nSize;

    for(i=1;i<nNodes;i++)
    {
        pNode->next = (tNode*)pMemPool;
        pNode       = pNode->next;
        pMemPool    += nSize;
    }
    pNode->next = NULL;

    return pNode;
}

/**
 * @brief Delete tracked objects
 *
 * @details This function checks if a tracked object should be removed from the list
 *          and if yes returns the node back to free pool
 *
 * @param [in,out] ptListHead        Pointer to head of list
 * @param [in,out] ptListTail        Pointer to tail of list
 * @param [in,out] ptListFreeHead    Pointer to head of free pool
 * @param [in,out] ptListFreeTail    Pointer to tail of free pool
 * @param [in]     framerate         Frame rate
 *
 * @return
 * Always returns 0
 */
static
int CleanUpList(
                tNode **ptListHead,
                tNode **ptListTail,
                tNode **ptListFreeHead,
                tNode **ptListFreeTail,
                int   framerate
               )
{
    tNode *ptNodePrev;   
    tNode *ptNodeNext;   
    tNode *ptNode;

    ptNodePrev = NULL;
    ptNode     = *ptListHead;

    while(ptNode!=NULL)
    {
        ptNodeNext = ptNode->next;
        if(ptNode->pfNodeRemove(ptNode,framerate)) 
        {
           RemoveNode(ptListHead,    ptListTail,     ptNode, ptNodePrev); 
           InsertNode(ptListFreeHead,ptListFreeTail, ptNode);
        }
        else
        {
            ptNodePrev = ptNode;
        }
        ptNode = ptNodeNext;

    }
    return 0;
}

/**
 * @brief Add new entry to track list
 *
 * @details This function checks if an object from to be track list should be moved into
 *          track list. If an object in to be track list has been tracked continuously, then
 *          it is removed from the to be track list and added to track list
 *
 * @param [in,out] ptTrackListHead           Head of track list
 * @param [in,out] ptTrackListTail           Tail of track list
 * @param [in,out] ptToBeTrackListHead       Head of To Be Track List
 * @param [in,out] ptToBeTrackListTail       Tail of To Be Track List
 * @param [in,out] ptFreeTrackListHead       Head of Free Pool
 * @param [in,out] ptFreeTrackListTail       Tail of Free Pool
 * @param [in,out] ptFreeToBeTrackListHead   Head of To Be Free Pool
 * @param [in,out] ptFreeToBeTrackListTail   Tail of To Be Free Pool
 *
 * @return
 * Always returns 0
 */
static 
void AddToTrackList(
                    tTrackNode     **ptTrackListHead,
                    tTrackNode     **ptTrackListTail,
                    tToBeTrackNode **ptToBeTrackListHead,
                    tToBeTrackNode **ptToBeTrackListTail,
                    tTrackNode     **ptFreeTrackListHead,
                    tTrackNode     **ptFreeTrackListTail, 
                    tTrackNode     **ptFreeToBeTrackListHead,
                    tTrackNode     **ptFreeToBeTrackListTail,
                    uint16_t         *pDepthImg,
                    int16_t         *pPointCloudImg,
                    uint16_t         nWidth,
                    uint16_t         nHeight
                   )
{
    tNode *ptNodePrev;   
    tNode *ptNodeNext;   
    tNode *ptNode;
    tNode *ptNodeNew;

    ptNodePrev = NULL;
    ptNode     = (tNode*)*ptToBeTrackListHead;

    while(ptNode!=NULL)
    {
        ptNodeNext = ptNode->next;
        float min_count = (float)((ADI_TRACK_LIST_MATCH_COUNT)*gFrameRate);
        min_count = min_count > 2.0f ? min_count : 2.0f;
        if(ptNode->match_count >= (int) (min_count) ) 
        {
            /* remove from the TO Be Track list */
            RemoveNode((tNode**)ptToBeTrackListHead, (tNode**)ptToBeTrackListTail, ptNode, 
                ptNodePrev);

            /* allocate new node for track list */
            ptNodeNew = AllocNode((tNode**)ptFreeTrackListHead, (tNode**)ptFreeTrackListTail);

            if(ptNodeNew != NULL)
            {
                /* copy features from To be track node to track node */
                ptNodeNew->recency = 0; 
                ptNodeNew->total_count = 1;
                ptNodeNew->match_count = 1;
                ptNodeNew->min_x       = ptNode->min_x;
                ptNodeNew->min_y       = ptNode->min_y;
                ptNodeNew->max_x       = ptNode->max_x;
                ptNodeNew->max_y       = ptNode->max_y;
                ptNodeNew->area        = ptNode->area;
                ptNodeNew->centroid_x  = ptNode->centroid_x;
                ptNodeNew->centroid_y  = ptNode->centroid_y;
                ptNodeNew->depth_val = ptNode->depth_val;
                ptNodeNew->bPredicted = ptNode->bPredicted;
                ptNodeNew->PointCloud_x = ptNode->PointCloud_x;
                ptNodeNew->PointCloud_y = ptNode->PointCloud_y;
                ptNodeNew->PointCloud_z = ptNode->PointCloud_z;
                ptNodeNew->pfNodeRemove = IsTrackNodeRemove;
            
                ((tTrackNode*)ptNodeNew)->index       = 0;
                ((tTrackNode*)ptNodeNew)->start       = 0;

                ((tTrackNode*)ptNodeNew)->label       = gGlobalLabel;
                gGlobalLabel = gGlobalLabel >=  gMaxNumTrkNodes ? 0 : gGlobalLabel+1;

                ((tTrackNode*)ptNodeNew)->centroid[0][0] = (short)(ptNodeNew->centroid_x/(float)ptNodeNew->area);
                ((tTrackNode*)ptNodeNew)->centroid[1][0] = (short)(ptNodeNew->centroid_y/(float)ptNodeNew->area);

                uint16_t nCenX = (ptNodeNew->min_x + ptNodeNew->max_x) >> 1;
                uint16_t nCenY = (ptNodeNew->min_y + ptNodeNew->max_y) >> 1;
                int16_t n3dX, n3dY, n3dZ;
                Get3DPointCloud(pPointCloudImg, nWidth, nHeight,
                    nCenX, nCenY, &n3dX, &n3dY, &n3dZ);

                //Add 3d location
                ((tTrackNode*)ptNodeNew)->PointCloudindex = 0;
                ((tTrackNode*)ptNodeNew)->PointCloudstart = 0;
                ((tTrackNode*)ptNodeNew)->PointCloudcentroid[0][0] = n3dX;
                ((tTrackNode*)ptNodeNew)->PointCloudcentroid[1][0] = n3dY;
                ((tTrackNode*)ptNodeNew)->PointCloudcentroid[2][0] = n3dZ;

                ((tTrackNode*)ptNodeNew)->state = APPEAR;
                ((tTrackNode*)ptNodeNew)->nOccludingId = -1;

                /* add node to track list */
                InsertNode((tNode**)ptTrackListHead, (tNode**)ptTrackListTail, ptNodeNew);
            }
            else
            {
#ifdef DEBUG
                printf("Error: No memory in Free track nodes pool\n");
#else
                gnErrorCode = ADI_TRACK_ERROR_MEM_FREE_POOL;
#endif
            }

            /* add removed to be track node to free to be list */
            InsertNode((tNode**)ptFreeToBeTrackListHead, (tNode**)ptFreeToBeTrackListTail, ptNode);
        }
        else
        {
            ptNodePrev = ptNode;
        }
        ptNode = ptNodeNext;

    }
}

/**
 * @brief Compare object in frame to tracked objects
 *
 * @details This function compares each object in the tracked list with objects in
 *          current frame. If a match is found then the label id of the tracked object
 *          association with current object is updated. For example, lets say tracked
 *          object 2 is matched with object 1 in frame, then label id of object 1 is updated.
 *          It's possible that multiple objects may be matched to the same object in
 *          current frame.
 *          In the second pass the label association array is checked to find out multiple
 *          matches, and if there are any then conflicts are resolved to keep only the best match
 *
 * @param [in] ptNodeHead                Pointer to Head of Track List
 * @param [in] ptLabel                   Objects in current frame
 * @param [in] numLabels                 Number of objects in current frame
 * @param [in] label_assoc               Label association for current objects
 * @param [in] label_assoc_tracklist     Label association for track list objects
 *
 * @return
 * Always returns 0
 */
static
int CompareLabelWithTrackNode(
                              tNode            *ptNodeHead,
                              ObjFeat          *ptLabel,
                              int              numLabels,
                              int              *label_assoc,
                              int              *label_assoc_tracklist,
                              int16_t         *p3dImage,
                              uint16_t         nImageWidth,
                              uint16_t         nImageHeight,
                              int              nThreshold,
                              bool             bUse3DTracking
                             )
{
    tCompare Object1;
    tCompare Object2;
    tCompare ObjectTemp;
    int i;
    int objectlabel_id;
    int temp_match;
    ObjFeat *ptLabelCurrentFrame;
    tNode  *ptNode;
    tNode  *ptNodeTemp;

    ptNode = ptNodeHead;

    while(ptNode != NULL)
    {
        ptLabelCurrentFrame = ptLabel;
        ptNode->objectlabel_id = -1;

        CopyFromNode(&Object1,ptNode, p3dImage, nImageWidth, nImageHeight);
        /* Find Centroid, size*/
        CalculateObjectProperties(&Object1);

        for(i=0;i<numLabels;i++)
        {
            //if the object ptLabelCurrentFrame[i] state is predicted, continue here so we dont match
            if (ptLabelCurrentFrame[i].detection_type == PREDICTED)
            {
                continue;
            }

            //printf("labels = %d\n", i);
            if(label_assoc_tracklist[i] > 0)
            {
                continue;
            }
            /* Find match */        
            CopyFromObjFeat(&Object2,ptLabelCurrentFrame,i, p3dImage, nImageWidth, nImageHeight);
             /* Find Centroid, size*/           
            CalculateObjectProperties(&Object2);

            /* Find match */
            if(CompareObjects(&Object1,&Object2, nThreshold, bUse3DTracking))
            {
                if(ptNode->objectlabel_id == -1)
                {
                    /* First Match */
                    ptNode->objectlabel_id = i;
                    //ptLabelCurrentFrame[i].ptr = (void*)ptNode;        
                }
                else
                {
                    objectlabel_id = ptNode->objectlabel_id;
                    /* compare current match with previous match to find best match */ 
                    /* for current label */
                    CopyFromObjFeat(&ObjectTemp,ptLabelCurrentFrame,objectlabel_id, p3dImage, nImageWidth, nImageHeight);
                    CalculateObjectProperties(&ObjectTemp);
                    
                    //Based on distance find the best match
                    if(ConflictResolve(&Object1, &Object2, &ObjectTemp, bUse3DTracking))
                    {
                        /* Replace with the better match */
                        //ptLabelCurrentFrame[ptNode->objectlabel_id].ptr = NULL;        
                        ptNode->objectlabel_id = i;
                        //ptLabelCurrentFrame[i].ptr = (void*)ptNode;        
                    }
                    
                } 
            }
        }       
        /* Temp storage to help in second pass */
        if(ptNode->objectlabel_id != -1)
        {
               label_assoc[ptNode->objectlabel_id ]++;
        }
        ptNode = ptNode->next;
    }

    /* First pass over. Now to remove instances when two members of the track list point to the same label */ 
     for(i=0;i<numLabels;i++)
    {
        if(label_assoc[i] > 1)
        {
            ptNode = ptNodeHead;
            temp_match = 0;
            ptLabelCurrentFrame = ptLabel;
            
            CopyFromObjFeat(&Object1, ptLabelCurrentFrame, i, p3dImage, nImageWidth, nImageHeight);
             /* Find Centroid, size*/
            CalculateObjectProperties(&Object1);

            ptNodeTemp = NULL;
            while(ptNode != NULL)
            {
                if(ptNode->objectlabel_id == i)
                {
                    if(temp_match)
                    {
                        label_assoc[i]--;

                        CopyFromNode(&Object2,ptNode, p3dImage, nImageWidth, nImageHeight);
                        /* Find Centroid, size*/
                        CalculateObjectProperties(&Object2);
                
                        CopyFromNode(&ObjectTemp,ptNodeTemp, p3dImage, nImageWidth, nImageHeight);
                        /* Find Centroid, size*/
                        CalculateObjectProperties(&ObjectTemp);
                        
                        if(ConflictResolve(&Object1, &Object2, &ObjectTemp, bUse3DTracking))
                        {
                            /*ptTrackNode is the better match*/
                            ptNodeTemp->objectlabel_id = -1;
                            ptNodeTemp = ptNode;
                            //ptLabelCurrentFrame[i].ptr = (void*)ptNode;
                        }
                        else
                        {
                            /*ptTrackNodeTemp is the better match*/
                            ptNode->objectlabel_id = -1;
                        } 
                    }
                    else
                    {
                        temp_match = 1;
                        ptNodeTemp = ptNode;
                        //ptLabelCurrentFrame[i].ptr = (void*)ptNode;
                        
                    }
                }
                ptNode = ptNode->next;    
            }
        }
    }

#if 1
    /* Now compare objects that are left with the predicted objects*/
    /* The location has to be updated from */
    ptNode = ptNodeHead;
    while (ptNode != NULL)
    {
        ptLabelCurrentFrame = ptLabel;

        //only do for nodes which are not matched already to detected labels
        if (ptNode->objectlabel_id != -1)
        {
            ptNode = ptNode->next;
            continue;
        }

        CopyFromNode(&Object1, ptNode, p3dImage, nImageWidth, nImageHeight);

        /* Find Centroid, size*/
        CalculateObjectProperties(&Object1);

        for (i = 0; i < numLabels; i++)
        {
            //if the object ptLabelCurrentFrame[i] state is predicted, continue here so we dont match
            if (ptLabelCurrentFrame[i].detection_type == DETECTED)
            {
                continue;
            }

            //printf("labels = %d\n", i);
            if (label_assoc_tracklist[i] > 0)
            {
                continue;
            }
            /* Find match */
            CopyFromObjFeat(&Object2, ptLabelCurrentFrame, i, p3dImage, nImageWidth, nImageHeight);
            /* Find Centroid, size*/
            CalculateObjectProperties(&Object2);

            /* Find match */
            if (CompareObjects(&Object1, &Object2, nThreshold, bUse3DTracking))
            {
                if (ptNode->objectlabel_id == -1)
                {
                    /* First Match */
                    ptNode->objectlabel_id = i;
                    //ptLabelCurrentFrame[i].ptr = (void*)ptNode;        
                }
                else
                {
                    objectlabel_id = ptNode->objectlabel_id;
                    /* dont override a match that is made with detected objects*/
                    if (ptLabelCurrentFrame[objectlabel_id].detection_type == PREDICTED)
                    {
                        /* compare current match with previous match to find best match */
                        /* for current label */
                        CopyFromObjFeat(&ObjectTemp, ptLabelCurrentFrame, objectlabel_id, p3dImage, nImageWidth, nImageHeight);
                        CalculateObjectProperties(&ObjectTemp);

                        //Based on distance find the best match
                        if (ConflictResolve(&Object1, &Object2, &ObjectTemp, bUse3DTracking))
                        {
                            /* Replace with the better match */
                            //ptLabelCurrentFrame[ptNode->objectlabel_id].ptr = NULL;        
                            ptNode->objectlabel_id = i;
                            //ptLabelCurrentFrame[i].ptr = (void*)ptNode;        
                        }
                    }
                }
            }
        }
        /* Temp storage to help in second pass */
        if (ptNode->objectlabel_id != -1)
        {
            label_assoc[ptNode->objectlabel_id]++;
            //if there is a match, update the properties of label using depth data
            //take bounding box dimensions from matched node. 
            //approx location from label estimate
            //use sad to find best match object that would work
        }
        ptNode = ptNode->next;
    }

    /* First pass over. Now to remove instances when two members of the track list point to the same label */
    for (i = 0; i < numLabels; i++)
    {
        if (label_assoc[i] > 1)
        {
            ptNode = ptNodeHead;
            temp_match = 0;
            ptLabelCurrentFrame = ptLabel;

            CopyFromObjFeat(&Object1, ptLabelCurrentFrame, i, p3dImage, nImageWidth, nImageHeight);
            /* Find Centroid, size*/
            CalculateObjectProperties(&Object1);

            ptNodeTemp = NULL;
            while (ptNode != NULL)
            {
                if (ptNode->objectlabel_id == i)
                {
                    if (temp_match)
                    {
                        label_assoc[i]--;

                        CopyFromNode(&Object2, ptNode, p3dImage, nImageWidth, nImageHeight);
                        /* Find Centroid, size*/
                        CalculateObjectProperties(&Object2);

                        CopyFromNode(&ObjectTemp, ptNodeTemp, p3dImage, nImageWidth, nImageHeight);
                        /* Find Centroid, size*/
                        CalculateObjectProperties(&ObjectTemp);

                        if (ConflictResolve(&Object1, &Object2, &ObjectTemp, bUse3DTracking))
                        {
                            /*ptTrackNode is the better match*/
                            ptNodeTemp->objectlabel_id = -1;
                            ptNodeTemp = ptNode;
                            //ptLabelCurrentFrame[i].ptr = (void*)ptNode;
                        }
                        else
                        {
                            /*ptTrackNodeTemp is the better match*/
                            ptNode->objectlabel_id = -1;
                        }
                    }
                    else
                    {
                        temp_match = 1;
                        ptNodeTemp = ptNode;
                        //ptLabelCurrentFrame[i].ptr = (void*)ptNode;

                    }
                }
                ptNode = ptNode->next;
            }
        }
    }
#endif

    /*
    ptNode = ptNodeHead;
    ptLabelCurrentFrame = ptLabel;
    while(ptNode!= NULL)
    {
        UpdateNode(ptNode,ptLabelCurrentFrame);
        ptNode = ptNode->next;    
    } 
    */
    return 0;
}


extern int frame_num;
/**
 * @brief Tracking algorithms
 *
 * @details This function runs the tracking algorithm to match current objects to tracked
 *          objects. There are two lists, tracking list and To Be Tracked List. Objects
 *          in current frame are matched to objects in tracked list (only unique matches).
 *          If a match is found then the tracked object properties are updated. If no
 *          match is found, then the Occlusion List is scanned to see if a merged object
 *          has split. If a match is found then the occlusion list and the tracked list are
 *          updated. If two objects have merged to form a new object, then that is decided
 *          next.
 *          The To Be Tracked list is scanned to find any current objects for which no association
 *          has been found yet. If any association is found then the object in the To Be Track list
 *          is updated with properties of the current object.
 *          The Track list is scanned to remove any objects which have disappeared and cannot be tracked
 *          anymore. The To Be Track list is also scanned to move any objects which need to be upgraded
 *          to Track List. Any objects which have disappeared from To Be Track list are also deleted during
 *          this scan.
 *
 * @param [in] ptLabel     Current Objects in frame
 * @param [in] numLabels   Number of Current Objects 
 * @param [in] ptObjTrack  Instance of Track module
 *
 * @return
 * Always returns 0
 */
int TrackerPrivate::ObjectTrackingList(
                       ObjFeat* ptLabel,
                       int numLabels,
                       ADI_TRACK_OBJ_STATUS* ptTrackObjectStatus
                      )
{
    int i;
    
    int *label_assoc;
    int *label_assoc_new;
       
    ObjFeat* ptLabelCurrentFrame;
    tNode *ptNode;
    float *pOvlpIndex;
   
    pOvlpIndex = ptTrackObjectStatus->pOvlpIndex;
    label_assoc = ptTrackObjectStatus->label_assoc;
    label_assoc_new = ptTrackObjectStatus->label_assoc_new;

#if 0
    if(frame_num >= 24)
    {
        printf("Debug\n");
    }
#endif

    for(i=0;i<numLabels;i++)
    {
        label_assoc[i]      = 0;
        label_assoc_new[i]  = 0;
        ptLabel[i].ptr = NULL;
    }

      /* Match labels in the track list */ 
     CompareLabelWithTrackNode((tNode*)ptTrackObjectStatus->ptTrackListHead, ptLabel, 
         numLabels, &label_assoc[0], &label_assoc_new[0], pPointCloudImg, nWidth, nHeight, 
         nMultFactorThreshold, bTrackIn3D);
        
    ptNode = (tNode*)ptTrackObjectStatus->ptTrackListHead;
    while(ptNode != NULL)
    {
        if(ptNode->objectlabel_id != -1)
        {
            ((tTrackNode*)ptNode)->state = TRACK;
            ptLabel[ptNode->objectlabel_id].ptr = (void*)ptNode;
        }    
        else
        {
            ((tTrackNode*)ptNode)->state = DISAPPEAR;
        }
        //printf("MC:%d R:%d TC:%d ST:%d\n", ptNode->match_count, ptNode->recency, ptNode->total_count, ((tTrackNode*)ptNode)->state);

        ((tTrackNode*)ptNode)->bSplit = 0;
        ptNode = ptNode->next;
    }
#if 0
    DecideTrackSplitBeforeMerge((tNode*)ptTrackObjectStatus->ptTrackListHead,
                             ptLabel,
                             numLabels,
                             &label_assoc[0],
                             pOvlpIndex);
#endif
    
   //DecideTrackMerge((tNode*)ptTrackObjectStatus->ptTrackListHead, ptLabel, &ptTrackObjectStatus->tOcclusionPool, numLabels, &label_assoc[0], pOvlpIndex);
    UpdateMatchedNodes((tNode*)ptTrackObjectStatus->ptTrackListHead, ptLabel, 
        pDepthImg, pPointCloudImg, nWidth, nHeight);
    UpdateLabelToTrackNode(ptTrackObjectStatus->ptTrackListHead, ptLabel, pPointCloudImg, nWidth, nHeight);

    //DecideTrackSplit((tNode*)ptTrackObjectStatus->ptTrackListHead, ptLabel, numLabels, &label_assoc[0], pOvlpIndex);

    /* Match labels not assigned to any item in the track list to items in tobetracked list */ 
    CompareLabelWithTrackNode((tNode*)ptTrackObjectStatus->ptToBeTrackListHead, 
        ptLabel, numLabels, &label_assoc_new[0], &label_assoc[0], pPointCloudImg, nWidth, nHeight,
        nMultFactorThreshold, bTrackIn3D);
    UpdateMatchedNodes((tNode*)ptTrackObjectStatus->ptToBeTrackListHead, ptLabel, 
        pDepthImg, pPointCloudImg, nWidth, nHeight);
    
    /* Remove entries in track list based on recency and match count */
    CleanUpList((tNode**)&ptTrackObjectStatus->ptTrackListHead,
                (tNode**)&ptTrackObjectStatus->ptTrackListTail,
                (tNode**)&ptTrackObjectStatus->ptFreeTrackListHead,
                (tNode**)&ptTrackObjectStatus->ptFreeTrackListTail,
                gFrameRate); 

    /* Remove entries in tobetrack list based on recency and match count */
    CleanUpList((tNode**)&ptTrackObjectStatus->ptToBeTrackListHead,
                (tNode**)&ptTrackObjectStatus->ptToBeTrackListTail,
                (tNode**)&ptTrackObjectStatus->ptFreeToBeListHead,
                (tNode**)&ptTrackObjectStatus->ptFreeToBeListTail,
                gFrameRate); 
    
    /* To add entries in ToBeTrack List to Track List */ 
    AddToTrackList((tTrackNode **)&ptTrackObjectStatus->ptTrackListHead,
                   (tTrackNode **)&ptTrackObjectStatus->ptTrackListTail,
                   (tToBeTrackNode **)&ptTrackObjectStatus->ptToBeTrackListHead,
                   (tToBeTrackNode **)&ptTrackObjectStatus->ptToBeTrackListTail,
                   (tTrackNode **)&ptTrackObjectStatus->ptFreeTrackListHead,
                   (tTrackNode **)&ptTrackObjectStatus->ptFreeTrackListTail,
                   (tTrackNode **)&ptTrackObjectStatus->ptFreeToBeListHead,
                   (tTrackNode **)&ptTrackObjectStatus->ptFreeToBeListTail,
                    pDepthImg, pPointCloudImg, nWidth,nHeight);
            

    /*To add the unmatched labels to tobetracklist */
    ptLabelCurrentFrame = ptLabel;    
    for(i=0;i<numLabels;i++)
    {
#if 1
        if(label_assoc[i] == 0 && label_assoc_new[i] == 0 && 
            ptLabelCurrentFrame[i].area > ptTrackObjectStatus->nMinObjSize )
#else
        /* New Decision for far/near camera view based size */
        if(label_assoc[i] == 0 && label_assoc_new[i] == 0 && 
            ((
              ptLabelCurrentFrame[i].area > ptObjTrack->nMinRectSize && 
              ptLabelCurrentFrame[i].centroid_y < ptObjTrack->nHeight*0.4
              )||
             (
              ptLabelCurrentFrame[i].area > 3*ptObjTrack->nMinRectSize && 
              ptLabelCurrentFrame[i].centroid_y >= ptObjTrack->nHeight*0.4
              )
             )
           )
#endif
        {
            ptNode = AllocNode((tNode**)&ptTrackObjectStatus->ptFreeToBeListHead,
                               (tNode**)&ptTrackObjectStatus->ptFreeToBeListTail);

            if(ptNode != NULL)
            {
                ptNode->recency = 0;
                ptNode->objectlabel_id = -1;
                ptNode->total_count = 1;
                ptNode->match_count = 1;
                ptNode->min_x       = ptLabelCurrentFrame[i].min_x;
                ptNode->min_y       = ptLabelCurrentFrame[i].min_y;
                ptNode->max_x       = ptLabelCurrentFrame[i].max_x;
                ptNode->max_y       = ptLabelCurrentFrame[i].max_y;
                ptNode->area        = ptLabelCurrentFrame[i].area;
                ptNode->centroid_x  = ptLabelCurrentFrame[i].centroid_x;
                ptNode->centroid_y  = ptLabelCurrentFrame[i].centroid_y;
                ptNode->depth_val   = CalculateDepth(pDepthImg, nWidth, nHeight,
                                       (ptNode->min_x + ptNode->max_x) >> 1,
                                       (ptNode->min_y + ptNode->max_y) >> 1,
                                       (ptNode->max_x - ptNode->min_x),
                                       (ptNode->max_y - ptNode->min_y));//average depth of centre window pixels;
                ptNode->bPredicted = (ptLabelCurrentFrame[i].detection_type == PREDICTED);
                ptNode->pfNodeRemove = IsToBeNodeRemove;

                uint16_t nCenX = (ptNode->min_x + ptNode->max_x) >> 1;
                uint16_t nCenY = (ptNode->min_y + ptNode->max_y) >> 1;
                int16_t n3dX, n3dY, n3dZ;
                Get3DPointCloud(pPointCloudImg, nWidth, nHeight,
                    nCenX, nCenY, &n3dX, &n3dY, &n3dZ);

                ptNode->PointCloud_x = n3dX;
                ptNode->PointCloud_y = n3dY;
                ptNode->PointCloud_z = n3dZ;

                InsertNode((tNode**)&ptTrackObjectStatus->ptToBeTrackListHead,
                           (tNode**)&ptTrackObjectStatus->ptToBeTrackListTail,
                           ptNode);
            }
            else
            {
                //Insert error code here
#ifdef TRACK_DEBUG                
                printf("\n Exceed ToBeTrack memory");
#else
                gnErrorCode = ADI_TRACK_ERROR_MEM_TOBE_TRACK_NODE;
#endif
            }
        }
    } 
#ifdef DEBUG
    debug_output_tracklist(gpToBeTrackNode,gpTrackNode,ptLabel,numLabels);
#endif
    return 0; 
}

#if 0
/**
 * @brief Runs the tracking algorithm
 *
 * @details This function runs the tracking algorithm.
 *
 * @param [in] ptLabel     Current Objects in frame
 * @param [in] numLabels   Number of Current Objects 
 * @param [in] ptObjTrack  Instance of Track module
 *
 * @return
 * Always returns 0
 */
#pragma section("adi_fast_prio2_code")
int ObjectTracking(
                   ObjFeat* ptLabel,
                   int numLabels,
                   ADI_TRACK_OBJ_STATUS* ptTrackObjectStatus
                  )
{

    gpTrackNodeHead     = ptTrackObjectStatus->ptTrackListHead;
    gpToBeTrackNodeHead = ptTrackObjectStatus->ptToBeTrackListHead;
    
    ObjectTrackingList(ptLabel, numLabels, ptObjTrack);
    
    return 0;
}
#endif

#if 0
/**
 * @brief  Initialize the Tracker instance
 *
 * @details This function initializes the tracker instance module, sets up memory
 *          allocation for all free pools, and other lists and arrays.
 *
 * @param [in] ptObjTrack     Tracker instance
 *
 * @return
 * Always returns 0
 */
#pragma section("adi_fast_prio2_code")
int ObjectTrackInit(TrackerPrivate *ptObjTrack)
{
    int i;
    tTrackNode *ptNode;
    ADI_TRACK_OBJ_STATUS *ptTrackObjectStatus;
    int MaxTrackNodes     = ptObjTrack->nConfigMaxTrackNodes;
    int MaxToBeTrackNodes = ptObjTrack->nConfigMaxToBeTrackedNodes;
    tOccPool* ptOccPool; 
    unsigned char* ptMemory;
    unsigned char* ptMemoryNonCache;

    ptTrackObjectStatus = ptObjTrack->ptTrackObjectStatus;
    ptTrackObjectStatus->ptFreeTrackListHead = NULL;
    ptTrackObjectStatus->ptFreeTrackListTail = NULL;
    ptTrackObjectStatus->ptFreeToBeListHead  = NULL;
    ptTrackObjectStatus->ptFreeToBeListTail  = NULL;
    ptTrackObjectStatus->ptTrackListHead     = NULL;
    ptTrackObjectStatus->ptTrackListTail     = NULL;
    ptTrackObjectStatus->ptToBeTrackListHead = NULL;
    ptTrackObjectStatus->ptToBeTrackListTail = NULL;
    ptOccPool  = &ptTrackObjectStatus->tOcclusionPool;
    
    ptMemory = ptObjTrack->ptMemory;
    
    ptTrackObjectStatus->ptFreeTrackListTail = (tTrackNode*)InitFreeList((tNode**)&ptTrackObjectStatus->ptFreeTrackListHead,
                                                            ptMemory,
                                                            MaxTrackNodes,
                                                            sizeof(tTrackNode)
                                                           );              
    ptMemory += MaxTrackNodes * sizeof(tTrackNode);

    ptMemoryNonCache = ptObjTrack->ptMemoryNonCache;
    ptNode =  ptTrackObjectStatus->ptFreeTrackListHead;

    /* Initialize the memory for trajectory */
    for(i = 0;i < MaxTrackNodes;i++)
    {
        ptNode->centroid[0] = (short*) ptMemoryNonCache;
        ptMemoryNonCache +=  sizeof(short) * MAX_NUM_TRAJECTORY_ENTRIES;
        ptNode->centroid[1] = (short*) ptMemoryNonCache;
        ptMemoryNonCache +=  sizeof(short) * MAX_NUM_TRAJECTORY_ENTRIES;

        ptNode = ptNode->next;
    }
        
    ptTrackObjectStatus->ptFreeToBeListTail  = (tToBeTrackNode*)InitFreeList((tNode**)&ptTrackObjectStatus->ptFreeToBeListHead, 
                                                                              ptMemory, 
                                                                              MaxToBeTrackNodes,
                                                                              sizeof(tToBeTrackNode));              
    ptMemory += MaxToBeTrackNodes * sizeof(tToBeTrackNode);

    ptOccPool->ptFreeTail  = (tOccNode*)InitFreeList((tNode**)&ptOccPool->ptFreeHead, 
                                                     ptMemory,
                                                     MaxTrackNodes,
                                                     sizeof(tOccNode));              
    ptMemory += MaxTrackNodes * sizeof(tOccNode);

    ptTrackObjectStatus->pOvlpIndex = (float*)ptMemory;
    /* array is of size [MAX_TRACK_NODES][MAX_OBJS] */
    ptMemory += sizeof(float) * MaxTrackNodes * ptObjTrack->nConfigMaxObjs;  

    ptObjTrack->label_assoc = (int*)ptMemory;
    ptMemory += sizeof(int) * ptObjTrack->nConfigMaxObjs;

    ptObjTrack->label_assoc_new = (int*)ptMemory;
    ptMemory += sizeof(int) * ptObjTrack->nConfigMaxObjs;

    gGlobalLabel = 0;
    gMaxNumTrkNodes = MaxTrackNodes;
    gMaxNumObjs = ptObjTrack->nConfigMaxObjs;
    gFrameRate   = ptObjTrack->nFrameRate; 

    ptObjTrack->nNumTrackedObjects = 0 ;
    
    return 0;      
}
#endif
#if 0
/* For Debugging */
#pragma section("sdram0_bank3_code_cache")
void display_tracklistnode(tTrackNode *ptNode)
{
    printf("\nmin_x:%d min_y:%d max_x:%d max_y:%d object_label:%d centroid_x:%f centroid_y:%f area:%d recency:%d",ptNode->min_x,ptNode->min_y,ptNode->max_x,ptNode->max_y,ptNode->objectlabel_id, (float)ptNode->centroid_x/ptNode->area, (float)ptNode->centroid_y/ptNode->area, ptNode->area, ptNode->recency);
}

#pragma section("sdram0_bank3_code_cache")
void display_tobelistnode(tToBeTrackNode *ptNode)
{
    printf("\nmin_x:%d min_y:%d max_x:%d max_y:%d object_label:%d  centroid_x:%f centroid_y:%f area:%d recency:%d",ptNode->min_x,ptNode->min_y,ptNode->max_x,ptNode->max_y,ptNode->objectlabel_id,(float)ptNode->centroid_x/ptNode->area, (float)ptNode->centroid_y/ptNode->area, ptNode->area, ptNode->recency);
}  

#pragma section("sdram0_bank3_code_cache")  
void display_label(TrackedObj *ptNode)
{
    printf("\nmin_x:%d min_y:%d max_x:%d max_y:%d  centroid_x:%f centroid_y:%f area:%d",ptNode->min_x,ptNode->min_y,ptNode->max_x,ptNode->max_y,(float)ptNode->centroid_x/ptNode->area, (float)ptNode->centroid_y/ptNode->area, ptNode->area);
}    

#pragma section("sdram0_bank3_code_cache")
void debug_output_tracklist(tToBeTrackNode *ptToBeTrackNode, tTrackNode *ptTrackNode,TrackedObj *ptLabel, int numLabels)
{
    int i;
    printf("\n Labels ");
    for(i=0;i<numLabels;i++)
    {
        display_label(&ptLabel[i]);
    } 
    
    printf("\nTo be Tracked List");
    while(ptToBeTrackNode!=NULL)
    {
        display_tobelistnode(ptToBeTrackNode);
        ptToBeTrackNode = ptToBeTrackNode->next;            
    }
    printf("\nTracked List");
    while(ptTrackNode!=NULL)
    {
        display_tracklistnode(ptTrackNode);
        ptTrackNode = ptTrackNode->next;            
    }
}
#endif

