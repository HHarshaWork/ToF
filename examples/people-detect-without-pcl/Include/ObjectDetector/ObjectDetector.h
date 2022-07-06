/******************************************************************************
Copyright(c) 2021 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.  By using this software you agree
to the terms of the associated Analog Devices Software License Agreement.
*******************************************************************************/

/*! \file ObjectDetector.h
*   \brief Object Detector header file
*    Header file that stores classes and enums used for Object Detector applications
*/

#ifndef OBJECTDETECTOR_H
#define OBJECTDETECTOR_H

#include <stdint.h>
#include <vector>

#if defined(WIN_SDK_STATICLIB)
#define DL_DETECT_API
#else
#if defined(DEEPLEARNINGSDK_EXPORTS)
#define DL_DETECT_API   __declspec(dllexport)
#else
#define DL_DETECT_API   __declspec(dllimport)
#endif
#endif

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

/*! \def MAX_PEOPLE_DETECTION
    \brief Max people detected by application.

    A macro that defines the maximum number of objects that can be detected in a frame.
*/
#define MAX_DETECTED_OBJECTS 16

/*! \def PERSON_CLASS
    \brief Object classified as person.

    A macro that defines the objects only classified as person class.
*/
#define PERSON_CLASS 0

/*! \def HEAD_CLASS
    \brief Object classified as head.

    A macro that defines the objects only classified as head class.
*/
#define HEAD_CLASS 1

using namespace cv::dnn;

/*! \enum DETECTOR_TYPE
    \brief Type of detector.

    Enumeration for the type of detector this object is.
*/
typedef enum DETECTOR_TYPE
{
    DETECTOR_TYPE_NONE,  /**< enum value DETECTOR_TYPE_NONE. */  
    PEOPLE_DETECTOR      /**< enum value PEOPLE_DETECTOR. */
}eObjectType;


/*! \struct DetectedObjectInfo
    \brief Attributes of detected object.

    This structure holds the attributes and properties of the detected object.
*/
typedef struct
{
    short nMinX;        /**< struct member nMinX. */  
    short nMinY;        /**< struct member nMinY. */
    short nMaxX;        /**< struct member nMaxX. */
    short nMaxY;        /**< struct member nMaxY. */
    short nId;          /**< struct member nId. */
    short nClass;       /**< struct member nClass. */
    short nConfidence;  /**< struct member nConfidence. */
} DetectedObjectInfo;

/*! \struct ObjectDetectorParams
    \brief Properties used for configuring object detector .

    This structure holds the configurable parameters for the object detector.
*/
struct ObjectDetectorParams
{
    float fPeopleConfThreshold; /**< struct member fPeopleConfThreshold. People Detection threshold */  
    float fHeadConfThreshold;   /**< struct member fHeadConfThreshold. Head Detection threshold */ 
    float fNMSThreshold;        /**< struct member fNMSThreshold. NMS threshold */ 
    int nInpWidth;              /**< struct member nInpWidth. width of input frame to detection */ 
    int nInpHeight;             /**< struct member nInpHeight. height of input frame to detection */ 
};

/*! \class ObjectDetector
    \brief A class used for detecting objects .

    This structure holds the members and functions used for object detection applications.
*/
class DL_DETECT_API ObjectDetector
{
private:
    float PeopleconfThreshold = 0.3f;     /**< class member PeopleconfThreshold. Threshold to detect people obj */ 
    float HeadconfThreshold = 0.5f;       /**< class member HeadconfThreshold. Threshold to detect head obj */ 
    float nmsThreshold = 0.4f;            /**< class member nmsThreshold. NMS Threshold  */ 
    int inpWidth = 1024;                  /**< class member inpWidth. Input width  */ 
    int inpHeight = 1024;                 /**< class member inpHeight. Input height  */ 
    eObjectType type = PEOPLE_DETECTOR;   /**< class member type. Type of object to detect @param eObjectType*/ 
    uint8_t* pIRImage;                    /**< class member type. IR Image*/ 
    int* classIds;                        /**< class member classIds. Detected class ID*/
    float* confidences;                   /**< class member confidences. Detected confidence*/
    int* boxes;                           /**< class member boxes. Detected bounding boxes*/
    int NumDetections;                    /**< class member NumDetections. Num of detections*/
public:

    /*! \fn ObjectDetector(eObjectType type);
        \brief Constructor initialized with \a type object.
        \param type The type of eObjectType to initialize to.
    */
    ObjectDetector(eObjectType type);

    /*! \fn ObjectDetector(eObjectType type, int width, int height);
    \brief Constructor initialized with \a type object and \a width and \a height of the input image.
    \param type The type of eObjectType to initialize to.
    \param width The width of the input frame.
    \param height The height of the input frame.
    */
    ObjectDetector(eObjectType type, int width, int height);

    /*! \fn ~ObjectDetector();
    \brief Destructor for the class.
    */
    ~ObjectDetector();

    /*! \fn void InitModule();
    \brief Initialize the detector module.
    *      Initialize the properties and attributes of object detector module
    */
    void InitModule();

    /*! \fn void ConfigModule(ObjectDetectorParams *oConfigParams)
    \brief ConfigModule the detector module with user parameters.
    \param oConfigParams The configuration parameters of type ObjectDetectorParams.
    *      Configure the object detector module parameters
    */
    void ConfigModule(ObjectDetectorParams *oConfigParams);

    /*! \fn void Detect()
    \brief Run object detection.
    *      Run the object detection module on input frame
    */
    void Detect();

    /*! \fn int GetDetections(DetectedObjectInfo* oDetections, int DetectionClass, float fConfThreshold)
    \brief Get detected objects and return count
    *      Load the detected objects of input class and above the threshold. Returns number of objects loaded
    \param oDetections Structure to hold detected object attributes.
    \param DetectionClass The class of detected objects to get.
    \param fConfThreshold The threshold above which to get the detected objects.    
    */
    int GetDetections(DetectedObjectInfo* oDetections, int DetectionClass, float fConfThreshold);

    /*! \fn void CloseModule()
    \brief Close object detection instance
    *      Close the detection instance
    */
    void CloseModule();

    /*! \fn void SetIRImage()
    \brief Sets Input IR Image pointer
    *      Sets Input IR Image pointer
    */
    void SetIRImage(uint8_t*);
};


#endif /* OBJECTDETECTOR_H */
