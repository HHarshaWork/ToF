/* 
******************************************************************************
Copyright (c), 2008-2015 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************

Title: object_merge_split.c

Description: Functions to perform tracking of detected objects

*****************************************************************************/

/*!
 * @file     object_merge_split.c
 *
 * @brief    Classify detected nodes
 *
 * @details  This file contains functions to help identify and classify detected objects
 *
 */
/*=============  I N C L U D E S   =============*/
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "object_track.h"

/*=============  D E F I N E S  =============*/
//#define DEBUG
#define MAX_LABELS             1000

/*=============  E X T E R N A L S  ============*/
extern 
tNode*              AllocNode(
                              tNode **tFreeListHead,
                              tNode **tFreeListTail
                             );

extern
void                InsertNode(
                               tNode **tFreeListHead,
                               tNode **tFreeListTail, 
                               tNode *pNode
                              );

extern 
int                 UpdateNode(
                               tNode            *pNode,
                               ObjFeat *ptLabelCurrentFrame
                              );

extern
void               CopyFromNode(
                                tCompare *ptObject,
                                tNode    *ptNode,
                                int16_t* p3dImage,
                                uint16_t nImageWidth,
                                uint16_t nImageHeight
                               );


extern
void              CalculateObjectProperties(tCompare *ptObject);


extern
void             CopyFromObjFeat(
                                 tCompare *ptObject,
                                 ObjFeat *ptNode,
                                 int i,
                                 int16_t* p3dImage,
                                 uint16_t nImageWidth,
                                 uint16_t nImageHeight
                                );


extern
int              ConflictResolve(
                                 tCompare *ptObject,
                                 tCompare *ptObjectMatch1,
                                 tCompare *ptObjectMatch2
                                );

extern int gnErrorCode;
extern int gMaxNumObjs;

/*=============  D A T A  =============*/
int gnOccLabel[MAX_LABELS];
int gOcclusionLabel = 0;
int gIndex = 0;

/*=============  C O D E  =============*/
/**
 * @brief Remove from Occlusion pool
 *
 * @details This function removes the current node from the pool of nodes in occlusion pool
 *
 * @param [in,out] ptOccPool            Pointer to structure holding nodes of occlusion pool
 * @param [in]     ptNode               Pointer to node to be removed
 *
 * @return
 * None
 */
static
void                RemoveFromOcclusionPool(
                                            tOccPool   *ptOccPool,
                                            tTrackNode *ptNode
                                           )
{
}

/**
 * @brief Add to Occlusion pool
 *
 * @details This function adds the current node to occlusion pool
 *
 * @param [in,out] ptOccPool            Pointer to structure holding nodes of occlusion pool
 * @param [in]     ptNode               Pointer to node to be added
 *
 * @return
 * None
 */
static
void                AddToOcclusionPool(
                                       tOccPool   *ptOccPool,
                                       tTrackNode *ptNode
                                      )
{
    int nOccludingId = ptNode->nOccludingId;
    int isNodeFound  = 0;
    tOccNode *ptOccNode = ptOccPool->ptHead;
    tOccNode *ptOccNodeTemp;
    tOccNode *ptOccNodeCurr;

    while(ptOccNode != NULL)
    {
        if(ptOccNode->nOccludingId == nOccludingId)
        {
            /* Parent Node found in Occlusion Pool */
            /* Allocate new node */
            ptOccNodeCurr = (tOccNode*)AllocNode(
                                  (tNode**)&ptOccPool->ptFreeHead, 
                                  (tNode**)&ptOccPool->ptFreeTail
                                 );
            /* Add new node to the list */
            InsertNode((tNode**)&(ptOccNode->ptHead), (tNode**)&(ptOccNode->ptTail),(tNode *) ptOccNodeCurr);

            ptOccNodeCurr->ptNode = (tNode*)ptNode;

            isNodeFound = 1;
            break;
            
        }
        ptOccNode = ptOccNode->ptNext;
    }

    if(!isNodeFound)
    {
        /* Alloc a new node*/
        ptOccNodeTemp = (tOccNode*)AllocNode(
                                  (tNode**)&ptOccPool->ptFreeHead, 
                                  (tNode**)&ptOccPool->ptFreeTail
                                 );
        

        /* Insert the new node in the parent list */
        InsertNode(
                   (tNode**)&ptOccPool->ptHead,
                   (tNode**)&ptOccPool->ptTail,
                   (tNode* )ptOccNodeTemp
                  );
        ptOccNodeTemp->ptHead       = NULL;
        ptOccNodeTemp->ptTail       = NULL;
        ptOccNodeTemp->nOccludingId = ptNode->nOccludingId;

        /* Alloc new node */
        ptOccNodeCurr = (tOccNode*)AllocNode(
                                  (tNode**)ptOccPool->ptFreeHead, 
                                  (tNode**)ptOccPool->ptFreeTail
                        );

        /* Insert the new node in the child list */
        InsertNode(
                   (tNode**)&ptOccNodeTemp->ptHead,
                   (tNode**)&ptOccNodeTemp->ptTail,
                   (tNode* )ptOccNodeCurr
                  );

        ptOccNodeCurr->ptNode       = (tNode*)ptNode;
    }
    
}

/**
 * @brief Determine overlap
 *
 * @details This function computes the overlap between all objects
 *
 * @param [in] ptNodeHead           Pointer to head of linked list holding all nodes
 * @param [in] ptLabel              Pointer to structure storing all nodes
 * @param [in] numLabels            Total objects detected
 * @param [out] aOvlpIndex          Array holding overlap indexes between objects
 *
 * @return
 * Always returns 0
 */
static
float               ComputeObjectOverlap(
                                         tNode            *ptNodeHead, 
                                         ObjFeat          *ptLabel,
                                         int              numLabels,
                                         float            *aOvlpIndex
                                        )
{
    tNode *ptNode;
    int i,j;
    int nOvlpW, 
        nOvlpH;
    int nOvlpArea;
    int size1;
    int size2;

    ptNode = ptNodeHead;
    
    j = 0;
    while(ptNode != NULL)
    {
        for(i=0; i < numLabels; i++)
        {
               nOvlpW = (MIN(ptNode->max_x, ptLabel[i].max_x)) - (MAX(ptNode->min_x, ptLabel[i].min_x));
               nOvlpW = MAX(nOvlpW, 0);

               nOvlpH = (MIN(ptNode->max_y, ptLabel[i].max_y) - MAX(ptNode->min_y, ptLabel[i].min_y));
               nOvlpH = MAX(nOvlpH, 0);

               nOvlpArea = nOvlpH * nOvlpW;

               size1 =  (ptLabel[i].max_y - ptLabel[i].min_y) * (ptLabel[i].max_x - ptLabel[i].min_x);
               size2 =  (ptNode->max_y - ptNode->min_y) * (ptNode->max_x - ptNode->min_x);
               if( 0.8*size2 <= size1 )
                    //aOvlpIndex[j][i] = nOvlpArea/(float)((ptNode->max_x - ptNode->min_x) * (ptNode->max_y - ptNode->min_y));
                    aOvlpIndex[j*gMaxNumObjs+i] = nOvlpArea/(float)((ptNode->max_x - ptNode->min_x) * (ptNode->max_y - ptNode->min_y));
               else
                    //aOvlpIndex[j][i] = 0;
                    aOvlpIndex[j*gMaxNumObjs+i] = 0;
        }
        ((tTrackNode*)ptNode)->j = j; 
        j++;
        ptNode = ptNode->next;

    }
    return 0;
}

/**
 * @brief Determine merge
 *
 * @details This function determines if the tracked objects merged
 *
 * @param [in] ptNodeHead           Pointer to head of linked list holding all nodes
 * @param [in] ptLabel              Pointer to structure storing all nodes
 * @param [in] ptOccPool            Pointer to objects in occlusion pool
 * @param [in] numLabels            Total objects detected
 * @param [in] label_assoc          Pointer to label storage index
 * @param [in] aOvlpIndex           Array storing overlap index
 *
 * @return
 * Always returns 0
 */
int                 DecideTrackMerge(
                                     tNode            *ptNodeHead,
                                     ObjFeat          *ptLabel,
                                     tOccPool         *ptOccPool,
                                     int              numLabels,
                                     int              *label_assoc,
                                     float            *aOvlpIndex,
                                     int16_t         *p3dImage,
                                     uint16_t         nImageWidth,
                                     uint16_t         nImageHeight
                                    )
{
    
    int i, j, k;
    tTrackNode  *ptNode;
    tTrackNode  *ptNodeTemp;
    int    nTrkLabel;
    int    isOcclusion;
    int    nOccludingId;
    float  fOvlp;


    ComputeObjectOverlap(ptNodeHead, ptLabel, numLabels, aOvlpIndex);

    ptNode = (tTrackNode*)ptNodeHead;
    j=0;
    while(ptNode != NULL)
    {
        /* Find if an object in track list is TRACKED or not */
        if(ptNode->state == DISAPPEAR)
        {
           isOcclusion = 0;
           nTrkLabel   = j;//ptNode->label;
           for(i=0; i < numLabels; i++)
           {
               /* do the objects overlap ? */
               //fOvlp = aOvlpIndex[nTrkLabel][i];
               fOvlp = aOvlpIndex[nTrkLabel*gMaxNumObjs+i];

               /* Track Object overlaps with a new object */
               if(fOvlp >= 0.5)
               {
                   /* find out if there is any overlap of the new object with another track object */
                   ptNodeTemp  = (tTrackNode*)ptNodeHead;
                   k = 0;
                   while(ptNodeTemp != NULL && isOcclusion == 0 )
                   {
                       if(ptNodeTemp != ptNode && ptNodeTemp->state != DISAPPEAR)
                       {
                            nTrkLabel = k; //ptNodeTemp->label;
                            fOvlp = aOvlpIndex[nTrkLabel*gMaxNumObjs+i];
                            if(fOvlp >= 0.5)
                            {
                                /* Size of ptNode and ptNodeTemp is almost same as new object i */
                                isOcclusion  = 1;
#if 1
                                if(ptNode->nOccludingId == -1 && ptNodeTemp->nOccludingId == -1)
                                {
                                    nOccludingId = gOcclusionLabel++;
                                }
                                else
                                {
                                    if(ptNodeTemp->nOccludingId == -1)
                                    {
                                        nOccludingId = ptNode->nOccludingId;
                                    }
                                    else
                                    {
                                        if(ptNode->nOccludingId == -1)
                                        {
                                            nOccludingId = ptNodeTemp->nOccludingId;
                                        }
                                        else
                                        {
                                            nOccludingId = MIN(ptNode->nOccludingId,
                                                                ptNodeTemp->nOccludingId);
                                        }
                                    }
                                }
    
#else
                                if(label_assoc[i] < 1)
                                {
                                    nOccludingId = ptNodeTemp->nOccludingId != -1 ? ptNodeTemp->nOccludingId : ((tTrackNode*)ptNodeTemp)->label;
                                    nOccludingId = ptNodeTemp->nOccludingId != -1 ? ptNodeTemp->nOccludingId : gOcclusionLabel++;
                                    ptLabel[i].ptr = (void*)ptNodeTemp;
                                }
                                else
                                {
                                    nOccludingId = ptNodeTemp->nOccludingId != -1 ? 
                                        ptNodeTemp->nOccludingId :
                                        ((tTrackNode*)(ptLabel[i].ptr))->label;
                                }
#endif

                                label_assoc[i] += 1;
                                label_assoc[i] += ptNodeTemp->state == DISAPPEAR;
                        
                                if(ptNodeTemp->state == DISAPPEAR && label_assoc[i] == -1)
                                {
#ifdef TRACK_DEBUG
                                    printf("Not Tracked\n");
#else
                                    gnErrorCode = ADI_TRACK_ERROR_MERGE_NO_TRACK;
#endif
                                }
                                
                                ptNode->objectlabel_id     = i;
                                ptNodeTemp->objectlabel_id = i;
                                
                                
                                if(ptNode->nOccludingId == -1)
                                {
                                    CopyFromNode(&(ptNode->oOcclObj),(tNode *) ptNode, p3dImage, nImageWidth, nImageHeight);
                                }
                                if(ptNodeTemp->nOccludingId == -1)
                                {
                                    CopyFromNode(&(ptNodeTemp->oOcclObj),(tNode *) ptNodeTemp, p3dImage, nImageWidth, nImageHeight);
                                }
                                //UpdateNode(ptNode, ptLabel);
                                //UpdateNode(ptNodeTemp, ptLabel);
                                
                                ptNodeTemp->nOccludingId   = nOccludingId;
                                ptNode->nOccludingId       = nOccludingId;
                                //AddToOcclusionPool(ptOccPool, ptNode);

                                /*if(ptNodeTemp->state == DISAPPEAR)
                                    AddToOcclusionPool(ptOccPool, ptNodeTemp);
                                */

                                ptNodeTemp->state = TRACK;
                                ptNode->state     = TRACK;
                            }
                            
                       }
                       ptNodeTemp = ptNodeTemp->next;
                       k++;
                   }
               }

               if(isOcclusion) 
               {
                   break;
               }
           } 
        }
        ptNode = ptNode->next;
        j++;
    }
    return 0;
}

/**
 * @brief Compare 2 objects
 *
 * @details This function determines if two objects have merged into one
 *
 * @param [in] ptObject1           Object 1 to be compared
 * @param [in] ptObject2           Object 2 to be compared
 *
 * @return
 * 0 - If objects are far apart<br>
 * 1 - If objects are nearby
 */
static
int CompareOcclusionObjects(
                            tCompare *ptObject1,
                            tCompare *ptObject2
                            )
{
    float ratio_object1 = ptObject1->size_x /(float)ptObject1->size_y;
    float ratio_object2 = ptObject2->size_x /(float)ptObject2->size_y;
    float ratio1 = ratio_object1/ratio_object2;
    float ratio2 = ratio_object2/ratio_object1;
    int return_value = 0;
    float x_var = (float)ptObject1->centroid_x_actual - ptObject2->centroid_x_actual;
    float y_var = (float)ptObject1->centroid_y_actual - ptObject2->centroid_y_actual;
    
    float distance = (x_var)*(x_var) + (y_var)*(y_var);

    int nOvlpW;
    int nOvlpH;
    int nOvlpArea;
    //Replace with distance condition later
    //if((ABS(ptObject1->centroid_x_actual - ptObject2->centroid_x_actual) < 40) && (ABS(ptObject1->centroid_y_actual - ptObject1->centroid_y_actual) < 40))

#if 0
    if(distance < 625) 
    {
        if(((ratio1 >= 0.5) && (ratio1 <= 1)) || ((ratio2 >= 0.5) && (ratio2 <= 1)))
        {
            return_value = 1;
        }
    }
    else
#endif
    {
        /* if overlaps by > 80% then also match */
        nOvlpW = (MIN(ptObject1->max_x, ptObject2->max_x)) - (MAX(ptObject1->min_x, ptObject2->min_x));
        nOvlpW = MAX(nOvlpW, 0);

        nOvlpH = (MIN(ptObject1->max_y, ptObject2->max_y) - MAX(ptObject1->min_y, ptObject2->min_y));
        nOvlpH = MAX(nOvlpH, 0);

        nOvlpArea = nOvlpH * nOvlpW;

        if(ptObject1->size_x * ptObject1->size_y < ptObject2->size_x * ptObject2->size_y)
        {
            if( (nOvlpArea >= 0.5 * ptObject1->size_x * ptObject1->size_y))
                
                return_value = 1;
        }
        else
        {
            if( (nOvlpArea >= 0.5 * ptObject2->size_x * ptObject2->size_y))
                
                return_value = 1;
        }

    }
    
    return (return_value);   
}

/**
 * @brief Match node to object
 *
 * @details This function determines if new node is already present among the objects
 *
 * @param [in] ptNodeToMatch        Pointer to node to be matched
 * @param [in] ptLabel              Pointer to structure storing all objects
 * @param [in] numLabels            Total objects detected
 * @param [in] label_assoc          Pointer to label storage index
 *
 * @return
 * 0 - If no match<br>
 * 1 - If there is a match
 */
static int MatchNodeWithNewObj(
                               tNode            *ptNodeToMatch,
                               ObjFeat          *ptLabel,
                               int              numLabels,
                               int              *label_assoc,
                               int16_t*        p3dImage,
                               uint16_t         nImageWidth,
                               uint16_t         nImageHeight
                              )
{
    int i;
    int objectlabel_id;

    tCompare Object1;
    tCompare Object2;
    tCompare ObjectTemp;
    
    tNode  *ptNode;

    ptNode = ptNodeToMatch;

    memcpy(&Object1,(&((tTrackNode*)ptNode)->oOcclObj), sizeof(tCompare));
    /* Find Centroid, size*/
    CalculateObjectProperties(&Object1);

    for(i=0;i<numLabels;i++)
    {
        
        if(label_assoc[i] > 0 && ((tTrackNode*)(ptLabel[i].ptr))->nOccludingId == -1)
        {
            continue;
        }
        
        
        /* Find match */        
        CopyFromObjFeat(&Object2,ptLabel,i, p3dImage, nImageWidth, nImageHeight);
        /* Find Centroid, size*/           
        CalculateObjectProperties(&Object2);

        /* Find match */
        if(CompareOcclusionObjects(&Object1,&Object2))
        {
            if(ptNode->objectlabel_id == -1)
            {
                /* First Match */
                ptNode->objectlabel_id = i;
            }
            else
            {
                objectlabel_id = ptNode->objectlabel_id;
                /* compare current match with previous match to find best match */ 
                /* for current label */
                CopyFromObjFeat(&ObjectTemp,ptLabel,objectlabel_id, p3dImage, nImageWidth, nImageHeight);
                CalculateObjectProperties(&ObjectTemp);
                    
                /* Based on distance find the best match */
                if(ConflictResolve(&Object1, &Object2, &ObjectTemp))
                {
                    /* Replace with the better match */
                    ptNode->objectlabel_id = i;    
                }
                    
             }
        }
    }
    
    if(ptNode->objectlabel_id == -1)
        return 0;
    else
    {
        //assert(label_assoc[ptNode->objectlabel_id] != -1);
        return 1;
    }
}

#if 0
/**
 * @brief Decide if merged objects have split
 *
 * @details This function compares current objects to find if a merged object has
 *          split into objects in current frame
 *
 *
 * @param [in] ptNodeHead           Pointer to head of linked list holding all nodes
 * @param [in] ptLabel              Pointer to structure storing all objects
 * @param [in] numLabels            Total objects detected
 * @param [in] label_assoc          Pointer to label storage index
 * @param [in] aOvlpIndex           Array storing overlap index
 *
 * @return
 * Always returns 0
 */
int                 DecideTrackSplitBeforeMerge(
                                                tNode            *ptNodeHead,
                                                ObjFeat          *ptLabel,
                                                int              numLabels,
                                                int              *label_assoc,
                                                //float          aOvlpIndex [][ADI_MAX_NUM_OBJECTS]
                                                float            *aOvlpIndex,
                                                int16_t         *p3dImage,
                                                uint16_t         nImageWidth,
                                                uint16_t         nImageHeight
                                               )
{
    int i, j;
    int         nId;
    int         isFound;
    tTrackNode  *ptNode;
    tTrackNode  *ptNodeTemp;
    int         nMatch1;
    int         nMatch2;
    int         prev_label;
    int         *nOccLabel;


    nOccLabel = &gnOccLabel[0];
    gIndex = 0;
    /* For Merged objects find match */
    ptNode = (tTrackNode*)ptNodeHead;

    while(ptNode != NULL)
    {
        if(ptNode->nOccludingId != -1 && ptNode->state == DISAPPEAR)
        {
            /* find if there is a label which matches with 
             * the pre-merge state */
            nMatch1 = MatchNodeWithNewObj((tNode *) ptNode,
                                           ptLabel,
                                          numLabels,
                                          label_assoc, 
                                          p3dImage, 
                                          nImageWidth, 
                                          nImageHeight);
            
            
            if(nMatch1)
            {
                /*find if there is another node in the same merge pool (same occlusion id)
                 which also matches a label
                 1. Node is already matched - good
                 2. Node not already matched - find match
                 On match remove them from occlusion
                 */
                ptNodeTemp =(tTrackNode *) ptNodeHead;
                while(ptNodeTemp != NULL)
                {
                    nMatch2 = 0;
                    prev_label = ptNodeTemp->objectlabel_id;
                    if(ptNodeTemp != ptNode && 
                        ptNodeTemp->nOccludingId == ptNode->nOccludingId &&
                        ptNodeTemp->state == TRACK)
                    {
                        //assert(ptNodeTemp->nOccludingId != -1);
                        if(ptNodeTemp->state == TRACK)
                        {
                            nMatch2 = 1;
                            
                            /* Find new label which matches with pre-occlusion state */
                            MatchNodeWithNewObj((tNode *) ptNodeTemp,
                                                ptLabel,
                                                numLabels,
                                                label_assoc, 
                                                p3dImage, 
                                                nImageWidth, 
                                                nImageHeight);
                        }
                        else
                        {
                            nMatch2 = MatchNodeWithNewObj((tNode *)ptNodeTemp,
                                                         ptLabel,
                                                         numLabels,
                                                         label_assoc,                                                
                                                         p3dImage,
                                                         nImageWidth,
                                                         nImageHeight);
                        }
                    }
                    if(nMatch2)
                    {
                        break;
                    }
                    ptNodeTemp->objectlabel_id = prev_label;
                    ptNodeTemp = ptNodeTemp->next;
                }

                /* if 2 objects found in occlusion pool which match */
                if(nMatch2)
                {
                    //assert(ptNode->objectlabel_id == ptNodeTemp->objectlabel_id);
                    if(ptNodeTemp->state == TRACK)
                        label_assoc[prev_label]--;
                    label_assoc[ptNode->objectlabel_id]++;
                    label_assoc[ptNodeTemp->objectlabel_id]++;
                    ptLabel[ptNode->objectlabel_id].ptr = (void*)ptNode;
                    ptLabel[ptNodeTemp->objectlabel_id].ptr = (void*)ptNodeTemp;
                    ((tTrackNode*)ptNode)->bSplit = 1;
                    ((tTrackNode*)ptNodeTemp)->bSplit = 1;
                    ptNode->state = TRACK;
                    ptNodeTemp->state = TRACK;
                }
                else
                {
                    ptNode->objectlabel_id = -1;
                    ((tTrackNode*)ptNode)->bSplit = 0;
                }
            }
        }
        ptNode = ptNode->next;
    }
    /*    
    ptNode = ptNodeHead;

    
    while(ptNode != NULL)
    {
        if(ptNode->nOccludingId != -1 && ((tTrackNode*)ptNode)->bSplit == 1)
        {
            ptNode->nOccludingId = -1;
            //UpdateNode(ptNode, ptLabel);
        }
        ptNode = ptNode->next;
    }
    */
    for(i=0; i < numLabels; i++)
    {
        if(label_assoc[i] > 1)
        {
            /* more than one object forms a merge. They need to be merged back */
            ptNode =(tTrackNode *) ptNodeHead;
            ptNodeTemp = NULL;
            while(ptNode != NULL)
            {
                //assert(!(ptNode->nOccludingId == -1 && ptNode->objectlabel_id == i));
                if(ptNode->nOccludingId != -1 && ptNode->objectlabel_id == i)
                {
                    /* Get Occluding Label */
                    isFound = 0;
                    if(ptNodeTemp == NULL)
                    {
                        for(j=0; j < gIndex && !isFound ; j++)
                        {
                            if(nOccLabel[j] == ptNode->nOccludingId)
                                isFound = 1;
                        }
                        nId = isFound ? gOcclusionLabel++: ptNode->nOccludingId;
                        nOccLabel[gIndex] = ptNode->nOccludingId;
                        gIndex++;
                    }
                    else
                    {
                        nId = ptNodeTemp->nOccludingId;
                    }
                    ptNode->nOccludingId = nId;
                    ptNode->bSplit = 0;
                    ptNodeTemp = ptNode;
                }
                ptNode = ptNode->next;
            }
            
        }
    }
    ptNode = (tTrackNode *)ptNodeHead;
    while(ptNode != NULL)
    {
        if(ptNode->nOccludingId != -1 && ((tTrackNode*)ptNode)->bSplit == 1)
        {
            ptNode->nOccludingId = -1;
            //UpdateNode(ptNode, ptLabel);
        }
        ptNode = ptNode->next;
    }

    return 0;
}
#endif    

#if 0
int                 DecideTrackSplitAfterMerge(
                                               tNode            *ptNodeHead,
                                               ObjFeat *ptLabel,
                                               int              numLabels,
                                               int              *label_assoc,
                                               //float          aOvlpIndex [][ADI_MAX_NUM_OBJECTS]
                                               float            *aOvlpIndex
                                              )
{
    int i, j, k;
    int         objectlabel_id;
    tTrackNode  *ptNode;
    tTrackNode  *ptNodeTemp;
    int         nMatch1;
    int         nMatch2;
    int         isOcclusion;
    int         nOccludingId;
    float       fOvlp;


    /* For unmatched labels find match */

    for(i=0; i < numLabels; i++)
    {
        if(label_assoc[i] == -1)
        {
            /* Find if there is a match with a tracked object 
               Matching criterion: Overlap > 50%
             */
            ptNode = (tTrackNode*)ptNodeHead;
            while(ptNode != NULL)
            {
                if(ptNode->state == TRACK && ptNode->nOccludingId != -1)
                {
                    nOvlpW = (MIN(ptNode->max_x, ptLabel[i].max_x)) - (MAX(ptNode->min_x, ptLabel[i].min_x));
                    nOvlpW = MAX(nOvlpW, 0);

                    nOvlpH = (MIN(ptNode->max_y, ptLabel[i].max_y) - MAX(ptNode->min_y, ptLabel[i].min_y));
                    nOvlpH = MAX(nOvlpH, 0);

                    nOvlpArea = nOvlpH * nOvlpW;

                    if(nOvlpArea >= 50)
                    {
                        nMatch1 = 1;
                        /* find another object with the same occlusion id */
                        ptNodeTemp = ptNodeHead;
                        while(ptNodeTemp != NULL)
                        {
                            if(ptNodeTemp->nOccludingId == ptNode->nOccludingId)
                            {
                            
                            }

                            ptNodeTemp = ptNodeTemp->next;
                        }

                    }
                }
                ptNode = ptNode->next;
            }
        }
    }
    ptNode = (tTrackNode*)ptNodeHead;

    while(ptNode != NULL)
    {
        if(ptNode->nOccludingId != -1 && ptNode->state == DISAPPEAR)
        {
            /* find if there is a label which matches with 
             * the pre-merge state */
            nMatch1 = MatchNodeWithNewObj(ptNode,
                                           ptLabel,
                                          numLabels,
                                          label_assoc);
            
            
            if(nMatch1)
            {
                /*find if there is another node in the same merge pool (same occlusion id)
                 which also matches a label
                 1. Node is already matched - good
                 2. Node not already matched - find match
                 On match remove them from occlusion
                 */
                label_assoc[ptNode->objectlabel_id] = 1;
                ptNodeTemp = ptNodeHead;
                while(ptNodeTemp != NULL)
                {
                    if(ptNodeTemp != ptNode && 
                        ptNodeTemp->nOccludingId == ptNode->nOccludingId)
                    {
                        //assert(ptNodeTemp->nOccludingId == -1);
                        if(ptNodeTemp->state == TRACK)
                        {
                            nMatch2 = 1;
                            /* TBD: Check if matches with pre-occlusion state */
                            //assert(label_assoc[ptNodeTemp->objectlabel_id] != 1);
                        }
                        else
                        {
                            nMatch2 = MatchNodeWithNewObj(ptNodeTemp,
                                                         ptLabel,
                                                         numLabels,
                                                         label_assoc);
                        }
                    }
                    if(nMatch2)
                    {
                        break;
                    }
                    ptNodeTemp = ptNodeTemp->next;
                }

                /* if 2 objects found in occlusion pool which match */
                if(nMatch2)
                {
                    label_assoc[ptNode->objectlabel_id] = 1;
                    if(ptNodeTemp->state == TRACK)
                    {
                        //assert(label_assoc[ptNodeTemp->objectlabel_id] != 1);
                    }
                    else
                    {
                        //assert(label_assoc[ptNodeTemp->objectlabel_id] != -1);
                        label_assoc[ptNodeTemp->objectlabel_id] = 1;
                    }
                    
                    ((tTrackNode*)ptNode)->bSplit = 1;
                    ((tTrackNode*)ptNodeTemp)->bSplit = 1;
                    ptNode->state = TRACK;
                    ptNodeTemp->state = TRACK;
                }
                else
                {
                    label_assoc[ptNode->objectlabel_id] = -1;
                    ptNode->objectlabel_id = -1;
                    ((tTrackNode*)ptNode)->bSplit = 0;
                }
            }
        }
        ptNode = ptNode->next;
    }
        
    ptNode = ptNodeHead;

    while(ptNode != NULL)
    {
        if(ptNode->nOccludingId != -1 && ((tTrackNode*)ptNode)->bSplit == 1)
        {
            ptNode->nOccludingId = -1;
            //UpdateNode(ptNode, ptLabel);
        }
        ptNode = ptNode->next;
    }
    return 0;
}
#endif