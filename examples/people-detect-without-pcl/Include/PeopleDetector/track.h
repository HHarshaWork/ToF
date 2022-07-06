/******************************************************************************
Copyright(c) 2021 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.  By using this software you agree
to the terms of the associated Analog Devices Software License Agreement.
*******************************************************************************/

/*! \file track.h
*   \brief Tracking API class
*    This file contains structures, macro defines used for integrating tracking module
*/

#ifndef __TRACK_H__
#define __TRACK_H__

/*=============  I N C L U D E S   =============*/
#include <fstream>
#include <iostream>

/*==============  D E F I N E S  ===============*/
/* Error codes for Tracking Module */
/*! \def ADI_TRACK_ERROR_NONE
	\brief Error codes for Tracking Module

	No error during tracking.
*/
#define ADI_TRACK_ERROR_NONE                   0

/*! \def ADI_TRACK_ERROR_ALLOC_NODE
	\brief Error codes for Tracking Module

	Error during allocating node.
*/
#define ADI_TRACK_ERROR_ALLOC_NODE             1

/*! \def ADI_TRACK_ERROR_MEM_TOBE_TRACK_NODE
	\brief Error codes for Tracking Module

	Memory error during tobe tracked node.
*/
#define ADI_TRACK_ERROR_MEM_TOBE_TRACK_NODE    2

/*! \def ADI_TRACK_ERROR_MEM_FREE_POOL
	\brief Error codes for Tracking Module

	Error ruring during freeing memory from pool.
*/
#define ADI_TRACK_ERROR_MEM_FREE_POOL          3

/*! \def ADI_TRACK_ERROR_TEMP_NODE_NULL
	\brief Error codes for Tracking Module

	Error as temp node is null.
*/
#define ADI_TRACK_ERROR_TEMP_NODE_NULL         4

/*! \def ADI_TRACK_ERROR_MERGE_NO_TRACK
	\brief Error codes for Tracking Module

	Error during merging no track.
*/
#define ADI_TRACK_ERROR_MERGE_NO_TRACK         5

/*=============  E X T E R N A L S  ============*/
/*! \def WINDOW_SEARCH_STRIDE
	\brief Search window size for finding location
*/
#define WINDOW_SEARCH_STRIDE                  32 

/*! \def POINTCLOUD_WINDOW_SIZE
	\brief Pointcloud window size for averaging
*/
#define POINTCLOUD_WINDOW_SIZE                21
/*=============  D A T A    T Y P E S   =============*/

/*! \enum eState
	\brief Enumeration for Tracked Object State

	Enumeration for current tracked object state.
*/
typedef enum
{
    APPEAR = 0,   /* enum value APPEAR-State of object when it appears initially */
    DISAPPEAR,    /* enum value DISAPPEAR-State of object when it disappears */
    TRACK,        /* enum value TRACK-State of object when it is being tracked */
    MERGE,        /* enum value MERGE-State of object when it is merged with other objects */
    SPLIT         /* enum value SPLIT-State of obejct when it splits from a merged object */
}eState;

/*! \enum eDetType
	\brief Object detection state(reserved)

	Enumeration for current detected object state.
*/
typedef enum
{
    PREDICTED = 0,   /* enum value PREDICTED-Object is predicted */
    DETECTED         /* enum value DETECTED-Object is detected */
}eDetType;

/*! \struct ObjFeat
	\brief Struct to store detected object properties.

	Struct to attributes of detected object.
*/
typedef struct 
{
    short min_x;                /* x position of the top left corner of the rectangle which bounds the tracked object */
    short min_y;                /* y position of the top left corner of the rectangle which bounds the tracked object */
    short max_x;                /* x position of the bottom right corner of the rectangle which bounds the tracked object */
    short max_y;                /* y position of the bottom right corner of the rectangle which bounds the tracked object */
    unsigned int centroid_x;    /* x position of the centroid of the tracked object */
    unsigned int centroid_y;    /* y position of the centroid of the tracked object */
    int area;                   /* size of the tracked object */
    short depth_z;              /* Depth of the tracked object*/
    eDetType detection_type;       /* whether objected is detected (1) or predicted (0)*/
    void* ptr;
}ObjFeat;    

/*! \struct TrackedObj
	\brief Struct to store tracked object properties.

	Struct to attributes of tracked object.
*/
typedef struct 
{
    short min_x;                /* x position of the top left corner of the rectangle which bounds the tracked object */
    short min_y;                /* y position of the top left corner of the rectangle which bounds the tracked object */
    short max_x;                /* x position of the bottom right corner of the rectangle which bounds the tracked object */
    short max_y;                /* y position of the bottom right corner of the rectangle which bounds the tracked object */
    unsigned int centroid_x;    /* x position of the centroid of the tracked object */
    unsigned int centroid_y;    /* y position of the centroid of the tracked object */
    int area;                   /* size of the tracked object */
    int track_label;            /* unique id of the tracked object */
    int state;                  /* state of the tracked object */
    void *ptr;                  /* reserved */
    short *trkTrajectoryX;      /* x position of the trajectory (path) followed by the tracked object for the last N frames */
    short *trkTrajectoryY;      /* y position of the trajectory (path) followed by the tracked object for the last N frames */
    int   trkTrajectoryIndx;    /* index of the current frame into the tracking trajectory circular buffer */
    short* PointCloudTrajectoryX; /* x position of the 3d trajectory (path) followed by the tracked object for the last N frames */
    short* PointCloudTrajectoryY; /* y position of the 3d trajectory (path) followed by the tracked object for the last N frames */
    short* PointCloudTrajectoryZ; /* z position of the 3d trajectory (path) followed by the tracked object for the last N frames */
    int   PointCloudTrajectoryIndx;    /* index of the current frame into the 3d tracking trajectory circular buffer */
    int track_count;            /* tracked count until now */
    int total_count;            /* total count until now */
    bool bPredicted;            /* if node is predicted or detected*/
}TrackedObj;    

/*! \class Tracker
	\brief Tracker Class to support object tracking

	This class holds APIs to be used by the tracking application
*/
class Tracker
{
private:
    void* pPrivInstance;                 /**< class member pPrivInstance. */ 
public:
    int nErrorCode;                      /**< class member nErrorCode. Error code is any*/ 
    unsigned int nNumObjects;            /**< class member nNumObjects. Num input objects */ 
    unsigned int nNumTrackedObjects;     /**< class member nNumTrackedObjects. Num output tracked objects */ 

    /*! \fn Tracker(int nMaxObjects, int nMaxTimeTrackTrajectory)
    \brief Constructor for tracker class
    \param nMaxObjects Maximum no of objects to track 
    \param nMaxTimeTrackTrajectory Max trajectory points to store
    */
    Tracker(int nMaxObjects, int nMaxTimeTrackTrajectory);
    
    /*! \fn ~Tracker()
    \brief Destructor for tracker class
    */
    ~Tracker();
    
    /*! \fn bool IsInitialized()
    \brief Checks if all memories are initialized
    */
    bool IsInitialized();
    
    /*! \fn void Process(ObjFeat* objs, int numObjs, int frameNum)
    \brief Process the tracker state machine
    \param objs All detected objects to input to tracker 
    \param numObjs Num of detected objects to input to tracker        
    \param frameNum Current frame number
    */
    void Process(ObjFeat* objs, int numObjs, int frameNum);
    
    /*! \fn void GetTrackedObjects(TrackedObj** objs, int& numTrkObjs)
    \brief Get all tracked objects from the tracker
    \param objs All tracked objects in tracker of TrackedObj class
    \param numTrkObjs Number of tracked objects
    */
    void GetTrackedObjects(TrackedObj** objs, int& numTrkObjs);
    
    /*! \fn void SetFramerate(int framerate)
    \brief Set tracker frame rate 
    \param framerate Framerate to set in tracker 
    */    
    void SetFramerate(int framerate);
    
    /*! \fn void SetThresholdMultFactor(int fact)
    \brief Set search window for tracker
    \param fact Multiplying factor for search window 
    */    
    void SetThresholdMultFactor(int fact);
    
    /*! \fn void Enable3DTracking(bool bEnable)
    \brief Use 2D or 3D tracking
    \param bEnable Bool value to enable/disable 3D tracking
    */
    void Enable3DTracking(bool bEnable);

    /*! \fn void SetDepthImageDim(uint16_t nWidth, uint16_t nHeight)
    \brief Set the depth image dimensions
    \param nWidth Width of image
    \param nHeight Height of image
    */    
    void SetDepthImageDim(uint16_t nWidth, uint16_t nHeight);
    
    /*! \fn void SetDepthImage(uint16_t* pImage,int16_t* pPointCloudImage)
    \brief Set depth images to the tracker 
    \param pImage Depth image pointer
    \param pPointCloudImage Point cloud image pointer
    */    
    void SetDepthImage(uint16_t* pImage,int16_t* pPointCloudImage);
};

/*! \fn void FindHeadLocationUsingDepth(
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
\brief Find approximate head location using depth 
\param pDepthImage Input depth image
\param nImageWidth width of input depth image
\param nImageHeight height of input depth image
\param nCentreX nCentreX of detected object
\param nCentreY nCentreY of detected object
\param nBoxWidth Width of detected object
\param nBoxHeight Height of detected object
\param nMinX MinX of the search window to look for match for detected object
\param nMaxX MaxX of the search window to look for match for detected object
\param nMinY MinY of the search window to look for match for detected object
\param WindowStride Size of window stride
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
    uint16_t WindowStride);

#endif /*__TRACK_H__*/
