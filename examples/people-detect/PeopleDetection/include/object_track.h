/* 
******************************************************************************
Copyright (c), 2008-2015 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
/*!
 * @file     object_track.h
 *
 * @brief    Internal data structures and macro defines for tracking
 *
 * @details  This file contains structures, macro defines used internally by the tracking module 
 *
 */

#ifndef __OBJECTTRACK_H__
#define __OBJECTTRACK_H__

/*=============  I N C L U D E S   =============*/
#include <fstream>
#include <iostream>
#include "track.h"


/*==============  D E F I N E S  ===============*/
#define MAX_TRACK_NODES 200
#define MAX_TO_BE_TRACK_NODES 100

#define ABS(x) ((x) >= 0 ? (x) : (-(x)) )

#define DISTANCE(x1,x2,y1,y2) (((x1-x2)*(x1-x2)) + ((y1-y2)*(y1-y2)))

#ifndef MAX
#define MAX(x,y) (((x) > (y))? (x) : (y))
#endif

#ifndef MIN
#define MIN(x,y) (((x) < (y))? (x) : (y))
#endif

/* Number of seconds for track matching */
#define ADI_TRACK_LIST_MATCH_COUNT 0.2



/* Error codes for Tracking Module */
#define ADI_TRACK_ERROR_NONE                   0
#define ADI_TRACK_ERROR_ALLOC_NODE             1
#define ADI_TRACK_ERROR_MEM_TOBE_TRACK_NODE    2
#define ADI_TRACK_ERROR_MEM_FREE_POOL          3
#define ADI_TRACK_ERROR_TEMP_NODE_NULL         4
#define ADI_TRACK_ERROR_MERGE_NO_TRACK         5

#define DEPTH_COMPARISON_THRESHOLD           512     
#define SEARCH_WINDOW_PIXELS                  64
#define SEARCH_STRIDE                          8

#define INVALID_PIXEL                          0
#define WINDOW_SIZE                           21
//#define TRACK_DEBUG
/*=============  E X T E R N A L S  ============*/

/*=============  D A T A    T Y P E S   =============*/

/* Structure for Comparing Objects */
typedef struct __compare
{
    short min_x;               /* x position of the top left corner of the rectangle which bounds the tracked object */
    short min_y;               /* y position of the top left corner of the rectangle which bounds the tracked object */
    short max_x;               /* x position of the bottom right corner of the rectangle which bounds the tracked object */
    short max_y;               /* y position of the bottom right corner of the rectangle which bounds the tracked object */
    unsigned int centroid_x;   /* x position of the centroid of the tracked object */
    unsigned int centroid_y;   /* y position of the centroid of the tracked object */
    float centroid_x_actual;   /* actual x position of the centroid of the tracked object, after dividing by size */
    float centroid_y_actual;   /* actual y position of the centroid of the tracked object, after dividing by size */
    int area;                  /* size of the tracked object */
    short size_x;              /* width of the bounding box which encloses the tracked object */
    short size_y;              /* height of the bounding box which encloses the tracked object */
    short PointCloud_x;        /* 3d x location*/
    short PointCloud_y;        /* 3d y location*/
    short PointCloud_z;        /* 3d z location*/
}tCompare;

/* Structure for Label of Tracked Object */
typedef struct __label
{
    int nIndex;                /* Not used */
    int nTrackIndex;           /* Not used */
}tLabelIndex;

/* Structure for Node List */
typedef struct __tNode
{
    struct __tNode *next;       /* pointer to the next node in list */
    short min_x;                /* x position of the top left corner of the rectangle which bounds the tracked object */
    short min_y;                /* y position of the top left corner of the rectangle which bounds the tracked object */
    short max_x;                /* x position of the bottom right corner of the rectangle which bounds the tracked object */
    short max_y;                /* y position of the bottom right corner of the rectangle which bounds the tracked object */
    unsigned int centroid_x;    /* x position of the centroid of the tracked object */
    unsigned int centroid_y;    /* y position of the centroid of the tracked object */
    int area;                   /* size of the tracked object */
    int objectlabel_id;         /* unique tracked id for the object */
    int total_count;            /* number of frames for which this object is present in the list */
    int match_count;            /* number of frames for which this object is matched/tracked */
    int recency;                /* The last frame number when this object was tracked or matched */
    int depth_val;              /* average depth value of detected object*/
    bool bPredicted;            /* If node is detected or predicted */
    short PointCloud_x;         /* 3d x location*/
    short PointCloud_y;         /* 3d y location*/
    short PointCloud_z;         /* 3d z location*/
    int (*pfNodeRemove)(struct __tNode*, int);   /* Function pointer to remove the node from the list */
}tNode;

/* Structure for Nodes for To be Track List. Note that the first few entries are same as tNode */
typedef struct __ToBeTrackNode
{
    struct __ToBeTrackNode *next;  /* pointer to the next node in list */
    short min_x;                   /* x position of the top left corner of the rectangle which bounds the tracked object */
    short min_y;                   /* y position of the top left corner of the rectangle which bounds the tracked object */
    short max_x;                   /* x position of the bottom right corner of the rectangle which bounds the tracked object */
    short max_y;                   /* y position of the bottom right corner of the rectangle which bounds the tracked object */
    unsigned int centroid_x;       /* x position of the centroid of the tracked object */
    unsigned int centroid_y;       /* y position of the centroid of the tracked object */
    int area;                      /* size of the tracked object */
    int objectlabel_id;            /* unique tracked id for the object */
    int total_count;               /* number of frames for which this object is present in the list */
    int match_count;               /* number of frames for which this object is matched/tracked */
    int recency;                   /* The last frame number when this object was tracked or matched */
    int depth_val;                 /* average depth value of detected object*/
    bool bPredicted;               /* If node is detected or predicted */
    short PointCloud_x;            /* 3d x location*/
    short PointCloud_y;            /* 3d y location*/
    short PointCloud_z;            /* 3d z location*/
    int (*pfNodeRemove)(struct __tNode*, int);   /* Function pointer to remove the node from the list */
}tToBeTrackNode;

/* Structure for Histogram of Object */
typedef struct __histogram
{
    int nBin;                    /* Number of bins in histogram */
    int *count;                  /* Count for each bin */
}tHistogram;

/* Structure for Nodes for Track List. Note that the first few entries are same as tNode */
typedef struct __TrackNode
{
    struct __TrackNode *next;    /* pointer to the next node in list */
    short min_x;                 /* x position of the top left corner of the rectangle which bounds the tracked object */
    short min_y;                 /* y position of the top left corner of the rectangle which bounds the tracked object */
    short max_x;                 /* x position of the bottom right corner of the rectangle which bounds the tracked object */
    short max_y;                 /* y position of the bottom right corner of the rectangle which bounds the tracked object */
    unsigned int centroid_x;     /* x position of the centroid of the tracked object */
    unsigned int centroid_y;     /* y position of the centroid of the tracked object */
    int area;                    /* size of the tracked object */
    int objectlabel_id;          /* unique tracked id for the object */
    int total_count;             /* number of frames for which this object is present in the list */
    int match_count;             /* number of frames for which this object is matched/tracked */
    int recency;                 /* The last frame number when this object was tracked or matched */
    int depth_val;              /* average depth value of detected object*/
    bool bPredicted;            /* If node is detected or predicted */
    int (*pfNodeRemove)(struct __tNode*, int);  /* Function pointer to remove the node from the list */

    short *centroid[2];          /* pointer to tracking trajectory positions of x and y for the last N frames */
    short *PointCloudcentroid[3]; /* pointer to 3d tracking trajectory positions of x y and z for the last N frames */

    int index;                   /* index to the current position into the circular trajectory buffer */
    int start;                   /* base position of the circular trajectory buffer */

    int PointCloudindex;         /* index to the current position into the circular 3d trajectory buffer */
    int PointCloudstart;         /* base position of the 3d circular trajectory buffer */

    int label;                   /* tracking label */

    tHistogram histogram;        /* histogram of the tracked object */
    
    // To keep track of the label match 
    int object_id;               /* tracking id */

    eState state;                /* tracked state */

    int nOccludingId;            /* occlusion id */
    int bSplit;                  /* Is the object created by spliting */
    /*
    short min_occ_x;
    short min_occ_y;
    short max_occ_x;
    short max_occ_y;
    unsigned int centroid_occ_x;
    unsigned int centroid_occ_y;
    int occ_area;
    */
    tCompare oOcclObj;          /* properties of object when occluded for comparision to other objects in list */
    int j;
}tTrackNode;

/* Structure for Occlusion List */
typedef struct __OcclusionNode
{
    struct __OcclusionNode *ptNext;          /* Points to Next Occlusion Node */
    struct __OcclusionNode *ptHead;          /* Points to Head of Occlusion List */
    struct __OcclusionNode *ptTail;          /* Points to tail of Occlusion List */
    tNode                  *ptNode;          /* Points to current Occulusion Node */
    int                    nOccludingId;     /* Number which will be used for the next occlusion object */
}tOccNode;

/* Structure for Occlusion Lists, both free list and current list */
typedef struct __OcclusionPool
{
    tOccNode *ptFreeHead;                   /* Points to Head of Free list */
    tOccNode *ptFreeTail;                   /* Points to Tails of Free list */

    tOccNode *ptHead;                       /* Points to Head of Current List */
    tOccNode *ptTail;                       /* Points to Tail of Current List */
}tOccPool;

/* Structure for Current Tracking Lists and  Free Track Lists */
typedef struct __ObjectStatus
{
    tTrackNode *ptFreeTrackListHead;        /* Head of Free Track List Nodes */
    tTrackNode *ptFreeTrackListTail;        /* Tail of Free Track List Nodes */

    tToBeTrackNode *ptFreeToBeListHead;     /* Head of Free To be Track List Nodes */
    tToBeTrackNode *ptFreeToBeListTail;     /* Tail of Free To be Track List Nodes */
    
    tTrackNode *ptTrackListHead;            /* Head of Current Tracked Objects List */
    tTrackNode *ptTrackListTail;            /* Tail of Current Tracked Objects List */

    tToBeTrackNode *ptToBeTrackListHead;    /* Head of Current To Be Tracked Objects List */
    tToBeTrackNode *ptToBeTrackListTail;    /* Tail of Current To Be Tracked Objects List */

    float          *pOvlpIndex;             /* Overlap percentage of Objects */

    tOccPool       tOcclusionPool;          /* Occlusion List of Objects */

    int nMaxNodes;
    int* label_assoc;                  /* label association for the objects during tracking */
    int* label_assoc_new;              /* New label association for new objects which could not be matched to existng tracked objects */
    int nMinObjSize;                   /* Minimum Size of Object */
}ADI_TRACK_OBJ_STATUS;  

class TrackerPrivate
{
private:
    unsigned int nFrameRate;           /* Frame rate for tracking */
    unsigned int nMinRectSize;         /* Minimum bounding box size of object which will be considered for tracking */
    int nWidth;                        /* Width of the frame */
    int nHeight;                       /* Height of the frame */
    ObjFeat* ptObjFeat;                /* Objects properties set every frame before calling tracking */
    TrackedObj* ptTrackedObjects;      /* Tracked objects returned by tracking */
    ADI_TRACK_OBJ_STATUS* ptTrackObjectStatus;  /* Internal Tracking Lists */
    int nConfigMaxTrackNodes;          /* Maximum Track nodes. Set during configuration in application */
    int nConfigMaxToBeTrackedNodes;    /* Maximum To be Track nodes. Set during configuration in application */
    int nConfigMaxObjs;                /* Maximum Possible Objects in a frame. Set during configuration in application */
    int nErrorCode;                    /* Returned Error Code */
    unsigned int nNumObjects;          /* Number of Objects in the current frame which are to be tracked */
    unsigned int nNumTrackedObjects;   /* Number of Objects which could be tracked in the current frame */
    int nMinObjSize;                   /* Minimum Size of Object */
    unsigned char* ptMemTrack;
    unsigned char* ptMemToBeTrack;
    unsigned char* ptMemOcc;
    short* ptMemTrkTrajectory;
    short* pt3DMemTrkTrajectory;
    uint16_t* pDepthImg;
    int16_t* pPointCloudImg;
    int32_t  nMultFactorThreshold;
    bool     bTrackIn3D;
    int ObjectTrackingList(
        ObjFeat* ptLabel,
        int numLabels,
        ADI_TRACK_OBJ_STATUS* ptTrackObjectStatus
    );
public:
    TrackerPrivate(int nMaxObjects, int nMaxTimeTrackTrajectory);
    ~TrackerPrivate();
    bool IsInit();
    void Process(ObjFeat* objs, int numObjs);
    void GetTrackedObjects(TrackedObj** objs, int& numTrkObjs);
    void SetMinObjSize(int nMinSize)
    {
        nMinObjSize = nMinSize;
        ptTrackObjectStatus->nMinObjSize = nMinSize;
    }
    void SetDepthImage(uint16_t*pImg, int16_t* pPCImg)
    {
        pDepthImg = pImg;
        pPointCloudImg = pPCImg;
    }
    void SetDepthImageDim(uint16_t nImgWidth, uint16_t nImgHeight)
    {
        nWidth = nImgWidth;
        nHeight = nImgHeight;
    }
    void SetThresholdMultFactor(int32_t nMult)
    {
        nMultFactorThreshold = nMult;
    }
    void Enable3DTracking(bool bEnable)
    {
        bTrackIn3D = bEnable;
    }
    void SetFramerate(int nFramerate);
};

//Helper function to calculate average depth for a window around the centroid. 
uint16_t CalculateDepth(
    uint16_t* pDepthImage,
    uint16_t nImageWidth,
    uint16_t nImageHeight,
    int16_t  nCentreX,
    int16_t  nCentreY,
    int16_t  nWidthX,
    int16_t  nWidthY
);

bool FindHeadUsingDepth(
    uint16_t* pDepthImage,
    uint16_t nImageWidth,
    uint16_t nImageHeight,
    uint16_t* nCentreX,
    uint16_t* nCentreY,
    uint16_t nBoxWidth,
    uint16_t nBoxHeight,
    uint16_t RefDepth,
    uint16_t DepthThreshold,
    uint16_t WindowSize,
    uint16_t WindowStride);
#endif /*__OBJECTTRACK_H__*/

