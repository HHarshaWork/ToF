/******************************************************************************
Copyright(c) 2021 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.  By using this software you agree
to the terms of the associated Analog Devices Software License Agreement.
*******************************************************************************/

/*! \file PeopleDetector.h
*   \brief People Detector header file
*    Header file that stores classes and enums used for People Detector applications
*/

#ifndef PEOPLEDETECTOR_H
#define PEOPLEDETECTOR_H

#include <stdint.h>
#include <vector>
#include "ObjectDetector.h"
#include "track.h"
#if defined(WIN_SDK_STATICLIB)
#define DL_DETECT_API
#else
#if defined(DEEPLEARNINGSDK_EXPORTS)
#define DL_DETECT_API   __declspec(dllexport)
#else
#define DL_DETECT_API   __declspec(dllimport)
#endif
#endif

/*! \def MAX_PEOPLE_DETECTION
	\brief Max people detected by application.

	A macro that defines the maximum number of people that can be defined in a frame.
*/
#define MAX_PEOPLE_DETECTION                     16

/*! \def NUM_TRACK_POINTS
	\brief Max number of points to use for tracker.

	The maximum history to use for the tracker to hold for a tracked object.
*/
#define NUM_TRACK_POINTS                        100

/*! \enum Body
	\brief Enum of how much of a person to detect.

	Enumeration for the how much of a person to detect. Available options are upper body, lower body and full body.
*/
typedef enum
{
	UPPER_BODY = 0, /**< enum value UPPER_BODY. */ 
	LOWER_BODY = 1, /**< enum value LOWER_BODY. */ 
	FULL_BODY = 2,  /**< enum value FULL_BODY. */ 
}Body;

/*! \struct PointPixel
	\brief Struct to store 2D coordinates.

	Struct to store the 2D coordinates of the detected person.
*/
typedef struct
{
	short x; /**< struct value x. */ 
	short y; /**< struct value y. */ 
}PointPixel;

/*! \struct Point3DPixel
	\brief Struct to store 3D coordinates.

	Struct to store the 3D coordinates of the detected person.
*/
typedef struct
{
	short x; /**< struct value x. */ 
	short y; /**< struct value y. */ 
	short z; /**< struct value z. */ 
}Point3DPixel;

/*! \struct DetectedPeopleInfo
	\brief Struct to store attributes of detected person .

	Struct to store the properties of the detected person in frame.
*/
typedef struct
{
	PointPixel TopLeft;     /**< struct value TopLeft 2D coordinate of detected object. */
	PointPixel BottomRight; /**< struct value BottomRight 2D coordinate of detected object. */
	Point3DPixel nLocation; /**< struct value nLocation 3D coordinate of centre of detected object. */
	Body nBody;             /**< struct value @param Body Type of detected object. */
	int nLinkedIndex;       /**< struct value Index of correspoding linked head/body object. */
	int nLabelId;	        /**< struct value label id of detected object. */
	bool bMotionPresent;    /**< struct value whether the object was in motion (reserved for now). */
	bool bPredicted;        /**< struct value whether the object was detected or predicted (reserved for now). */
	PointPixel o2DTrajectory[NUM_TRACK_POINTS]; /**< struct value holding 2d trajectory of object. */
	Point3DPixel o3DTrajectory[NUM_TRACK_POINTS]; /**< struct value holding 3dtrajectory of object. */
	int nTrajectoryPoints;  /**< struct value stores num of tracjectory points present */
}DetectedPeopleInfo;

/*! \struct ConfigParamsPeopleDetector
	\brief Struct to store parameters to configure the people detector .

	Struct that holds configurable parameters to the properties of the people detector.
*/
typedef struct
{
	int   timeToStartCounting;              /**< struct value timeToStartCounting no of frames of no motion before counting. */
	float DistanceThresholdForStaticObjects;/**< struct value DistanceThresholdForStaticObjects 3d distance beyond which to classify object as moving. */
}ConfigParamsPeopleDetector;

/*! \class PeopleDetector
	\brief People Detection Class

	This class holds APIs to be used for people and head detection applications.
*/
class DL_DETECT_API PeopleDetector
{
private:
	int inpWidth;                          /**< class member inpWidth. Width of input frame */ 
	int inpHeight;                         /**< class member inpHeight. Height of input frame */ 
	int elementSize = 2;                   /**< class member elementSize. Size of 1 pixel */ 
	int maxPeopleDetection;                /**< class member maxPeopleDetection. Max num of people to detect */
	uint8_t* pImage;                       /**< class member pImage. Input image */
	uint8_t* pSilhouette;                  /**< class member pSilhouette. Output silhouette image */
	uint8_t* pIRImage;                    /**< class member pIRImage. Input IR 8 bit in RGB format image */
	uint16_t* pDepthImage;                 /**< class member pDepthImage. Input Depth image */
	int16_t*  pPointCloudImage;             /**< class member pPointCloudImage. Input 3D image */
	uint16_t* pFilteredDepthImage;         /**< class member pFilteredDepthImage. Output filtered depth image */
	uint8_t*  pLabelImageForSilloheute;     /**< class member pLabelImageForSilloheute. Output silhouette label image */
	ObjectDetector* pobjDet;               /**< class member pobjDet. @param ObjectDetector */
	DetectedPeopleInfo* poPeopleDetections;/**< class member poPeopleDetections. Detected people */
	DetectedPeopleInfo* poHeadDetections;  /**< class member poHeadDetections. Detected head*/
	
	Tracker* PeopleTracker;                /**< class member PeopleTracker. To track detected people objects*/
	bool bEnableTrackPeople = false;       /**< class member bEnableTrackPeople. Enable people tracking*/
	int numPplTracked = 0;                 /**< class member numPplTracked. Number of people tracked*/
	int numHeadTracked = 0;                /**< class member numHeadTracked. Number of heads tracked*/
	int frame_num = 0;                     /**< class member frame_num. Current frame number*/
	Tracker* HeadTracker;                  /**< class member HeadTracker. To track detected head objects*/
	bool bEnableTrackHead = false;         /**< class member bEnableTrackHead. Enable head tracking*/
	bool bEnableFullBodyTrack = false;     /**< class member bEnableFullBodyTrack. To enable head or upper body tracking*/

	ConfigParamsPeopleDetector params;    /**< class member params @param ConfigParamsPeopleDetector. 
										  Configurable parameters for the people detector*/
	 
	int numPplDetected;                    /**< class member numPplDetected. Number of people detected*/
	int numHeadDetected;                   /**< class member numHeadDetected. Number of head detected*/

	/*! \fn void FilterDepth()
	\brief Filter the input depth image.
	*/
	void FilterDepth();

	/*! \fn void SegmentUsingMeanandVariance(DetectedObjectInfo* objDet)
	\brief Segment object using mean and variance values of depth
	\param objDet The details of detected object
	*/
	void SegmentUsingMeanandVariance(DetectedObjectInfo* objDet);

	/*! \fn void SegmentUsingDepthRange(DetectedObjectInfo* objDet)
	\brief Segment object using depth range values
	\param objDet The details of detected object
	*/
	void SegmentUsingDepthRange(DetectedObjectInfo* objDet);

	/*! \fn void SetDefaultParams()
	\brief Set the default parameters of the detector
	*/
	void SetDefaultParams();


	/*! \fn void SetDefaultParams()
	\brief Set the configured parameters from @param ConfigParamsPeopleDetector to the detector
	*/
	void SetParams();

	/*! \fn cv::Mat GetSilhouettePeople(DetectedObjectInfo* objDet, int index)
	\brief Get the silhouette mask of detected object. Return type is opencv Mat
	\param objDet The details of detected object of which to extract the silhoutte
	\param index Index of the detected object array
	*/
	cv::Mat GetSilhouettePeople(DetectedObjectInfo* objDet, int index);

	/*! \fn cv::Mat PeopleDetector::GetSilhouettePeopleFromHead(DetectedObjectInfo* headObjDet,
		DetectedObjectInfo* objDet, int index, int headIndex)
	\brief Get the silhouette from head detected object. Return type is opencv Mat
	\param headObjDet The details of detected head of which to extract the silhoutte
	\param objDet The details of detected object of which to extract the silhoutte
	\param index Index of the detected person object
	\param headIndex Index of the detected head
	*/
	cv::Mat PeopleDetector::GetSilhouettePeopleFromHead(DetectedObjectInfo* headObjDet,
		DetectedObjectInfo* objDet, int index, int headIndex);

	/*! \fn DetectedPeopleInfo* GetHeadFromSilhouette()
	\brief Get the predicted head location of detected object from silhouette. 
	 Return the head properties
	*/
	DetectedPeopleInfo* GetHeadFromSilhouette();

	/*! \fn DetectedPeopleInfo* GetHeadFromSilhouette()
	\brief Get the predicted head location of detected object from silhouette.
	\param objDet The details of detected object of which to extract the silhoutte
	 Return the predicted head properties
	*/
	DetectedObjectInfo GetHeadFromRoi(DetectedObjectInfo objDet);
public:

	/*! \fn PeopleDetector();
	\brief Constructor for people detector.
    */
	PeopleDetector()
	{
		pImage = NULL;
		pIRImage = NULL;
		pDepthImage = NULL;
		pLabelImageForSilloheute = NULL;
		inpWidth = 1024;
		inpHeight = 1024;
		numPplDetected = 0;
		numPplTracked = 0;
		frame_num = 0;
		maxPeopleDetection = MAX_PEOPLE_DETECTION;
		poPeopleDetections = new DetectedPeopleInfo[maxPeopleDetection];
		poHeadDetections = new DetectedPeopleInfo[maxPeopleDetection];
		eObjectType type = PEOPLE_DETECTOR;
		pobjDet = new ObjectDetector(type, inpWidth, inpHeight);
		pobjDet->InitModule();

		pIRImage = new uint8_t[3 * inpWidth * inpHeight];
		pDepthImage = new uint16_t[inpWidth * inpHeight];
		pFilteredDepthImage = new uint16_t[inpWidth * inpHeight];
		pSilhouette = new uint8_t[inpWidth * inpHeight];
		pPointCloudImage = new int16_t[3 * inpWidth * inpHeight];		
		pLabelImageForSilloheute = new uint8_t[inpWidth * inpHeight];
		PeopleTracker = new Tracker(maxPeopleDetection, NUM_TRACK_POINTS);
		HeadTracker = new Tracker(maxPeopleDetection, NUM_TRACK_POINTS);
		SetDefaultParams();

		numHeadDetected = 0;
		numPplDetected = 0;
		bEnableFullBodyTrack = true;
	}

	/*! \fn PeopleDetector();
	\brief Constructor for people detector for input height and width.
	\param int Width of input image
	\param int Height of input image		
	*/
	PeopleDetector(int width, int height)
	{
		pImage = NULL;
		pIRImage = NULL;
		pDepthImage = NULL;
		pLabelImageForSilloheute = NULL;
		inpWidth = width;
		inpHeight = height;
		maxPeopleDetection = MAX_PEOPLE_DETECTION;
		numPplDetected = 0;
		numPplTracked = 0;
		frame_num = 0;
		poPeopleDetections = new DetectedPeopleInfo[maxPeopleDetection];
		poHeadDetections = new DetectedPeopleInfo[maxPeopleDetection];
		eObjectType type = PEOPLE_DETECTOR;
		pobjDet = new ObjectDetector(type, inpWidth, inpHeight);
		pobjDet->InitModule();
		pIRImage = new uint8_t[3 * inpWidth * inpHeight];
		pDepthImage = new uint16_t[inpWidth * inpHeight];
		pFilteredDepthImage = new uint16_t[inpWidth * inpHeight];
		pPointCloudImage = new int16_t[3 * inpWidth * inpHeight];
		pSilhouette = new uint8_t[inpWidth * inpHeight];
		pLabelImageForSilloheute = new uint8_t[inpWidth * inpHeight];
		PeopleTracker = new Tracker(maxPeopleDetection, NUM_TRACK_POINTS);
		HeadTracker = new Tracker(maxPeopleDetection, NUM_TRACK_POINTS);
		SetDefaultParams();

		numHeadDetected = 0;
		numPplDetected = 0;
		bEnableFullBodyTrack = true;
	}

	/*! \fn ~PeopleDetector();
	\brief Destructor for people detector.
	*/
	~PeopleDetector()
	{
		if (pIRImage) delete[] pIRImage;
		if (pDepthImage) delete[] pDepthImage;
		if (pFilteredDepthImage) delete[] pFilteredDepthImage;
		if (pPointCloudImage) delete[] pPointCloudImage;		
		if (pSilhouette) delete[] pSilhouette;
		if (pLabelImageForSilloheute) delete[] pLabelImageForSilloheute;
		delete[] poPeopleDetections;
		delete[] poHeadDetections;
		delete pobjDet;
		delete PeopleTracker;
		delete HeadTracker;
	}

	/*! \fn bool ConfigModule();
	\brief Configure the people detector module. Returns if it has been succesfully configured. 
	*/
	bool ConfigModule();

	/*! \fn void InjectFrame(uint8_t* pImage)
	\brief Inject input frame into the people detector module.
	\param pImage uint8_t* Pointer to the input image
	*/
	void InjectFrame(uint8_t* pImage);

	/*! \fn DetectedPeopleInfo* DetectPeople(int &numPeople)
	\brief Get all the detected people in the frame. Returns pointer to detected objects
	\param numPeople Number of people detected
	*/
	DetectedPeopleInfo* DetectPeople(int &numPeople);

	/*! \fn DetectedPeopleInfo* DetectPeople(int &numPeople)
	\brief Get only the people who have been static for the configured time. Returns pointer to detected objects
	\param numPeople Number of people detected
	*/
	DetectedPeopleInfo* DetectStaticPeople(int& numPeople);
	
	/*! \fn DetectedPeopleInfo* DetectHead(int &numPeople)
	\brief Get all the detected head in the frame. Returns pointer to detected objects
	\param numPeople Number of heads detected
	*/
	DetectedPeopleInfo* DetectHead(int& numPeople);

	/*! \fn void GetLocationHead(int& numHead, int& numPeopleDet,
		DetectedPeopleInfo* oPeople, DetectedPeopleInfo* oHead)
	\brief Get all the detected head of people who have been static for the given time.
	\param numHead Number of heads detected
	\param numPeopleDet Number of people detected
	\param oPeople Properties of detected people
	\param oHead Properties of detected heads
	*/
	void GetLocationHead(int& numHead, int& numPeopleDet,
		DetectedPeopleInfo* oPeople, DetectedPeopleInfo* oHead);	
	
	/*! \fn uint8_t* GetSilhouette()
	\brief Get the silhoutte of the detected people in the room. Returns the pointer to the silhouette image
	*/
	uint8_t* GetSilhouette();

	/*! \fn void SetPeopleTracking(bool bEnable)
	\brief Enable/Disable tracking mode in people detection module
	\param bEnable bool parameter Enable/Disable tracking
	*/
	void SetPeopleTracking(bool bEnable)
	{
		bEnableTrackPeople = bEnable;
		bEnableTrackHead = bEnable;
	}

	/*! \fn void SetParams(ConfigParamsPeopleDetector*)
	\brief Configure the parameters in people detection module
	\param oParam ConfigParamsPeopleDetector parameters to set in people detection module
	*/
	void SetParams(ConfigParamsPeopleDetector* oParam);

	/*! \fn uint16_t* GetProcessedDepthImage()
	\brief Return the processed depth image. Returns pointer to the processed depth image
	*/
	uint16_t* GetProcessedDepthImage();

	/*! \fn uint16_t* GetProcessedIRImage()
	\brief Return the processed IR RGB 8 bit image. Returns pointer to the processed IR image
	*/
	uint8_t* GetProcessedIRImage();

	/*! \fn void CloseModule()
	\brief Close the people detection module
	*/
	void CloseModule();

	/*! \fn void Run()
	\brief Run the people detection module
	*/
	void Run();
};

#endif /* PEOPLEDETECTOR_H */
