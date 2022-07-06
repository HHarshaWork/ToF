#include <iostream>
#include <fstream>
#include "tensorflow/c/c_api.h"
#include "tensorflow_c_functions.h"
#include "ObjectDetector.h"
#include <memory>
#include <algorithm>
#include <cstddef>
#include <iterator>
#include <vector>
#include <assert.h>
#include <string.h>
#include <stdint.h>

#define IOU_THRESHOLD 0.4f

using namespace std;

std::unique_ptr<MySession> session;

ObjectDetector::ObjectDetector(eObjectType m_type)
{
    PeopleconfThreshold = 0.3f;
    HeadconfThreshold = 0.5f;
    nmsThreshold = 0.4f;
    inpWidth = 1024;
    inpHeight = 1024;
    type = m_type;
    classIds = new int[MAX_DETECTED_OBJECTS];
    confidences = new float[MAX_DETECTED_OBJECTS];
    boxes = new int[4 * MAX_DETECTED_OBJECTS];
    pIRImage = NULL;
    NumDetections = 0;
}

ObjectDetector::ObjectDetector(eObjectType m_type, int width, int height)
{
    PeopleconfThreshold = 0.3f;
    HeadconfThreshold = 0.5f;
    nmsThreshold = 0.4f;
    inpWidth = width;
    inpHeight = height;
    type = m_type;
    classIds = new int[MAX_DETECTED_OBJECTS];
    confidences = new float[MAX_DETECTED_OBJECTS];
    boxes = new int[4*MAX_DETECTED_OBJECTS];  
    pIRImage = NULL;
    NumDetections = 0;
}


ObjectDetector::~ObjectDetector()
{
    delete[] classIds;
    delete[] confidences;
    delete[] boxes;
}

void ObjectDetector::SetIRImage(uint8_t* pImg)
{
    pIRImage = pImg;
}

void ObjectDetector::InitModule()
{
    if (type == PEOPLE_DETECTOR)
    {
         // Load a model.
        session = std::unique_ptr<MySession>(my_model_load(NULL, "input_1",
            "filtered_detections/map/TensorArrayStack/TensorArrayGatherV3",
            "filtered_detections/map/TensorArrayStack_1/TensorArrayGatherV3",
            "filtered_detections/map/TensorArrayStack_2/TensorArrayGatherV3"));

    }
}

void ObjectDetector::ConfigModule(ObjectDetectorParams* oConfigParams)
{
    PeopleconfThreshold = oConfigParams->fPeopleConfThreshold;
    HeadconfThreshold = oConfigParams->fHeadConfThreshold;
    nmsThreshold = oConfigParams->fNMSThreshold;
    inpWidth = oConfigParams->nInpWidth;
    inpHeight = oConfigParams->nInpHeight;
}

void ObjectDetector::Detect()
{
    cv::Mat IR_frame(inpHeight, inpWidth, CV_8UC3);
    IR_frame.data = pIRImage;
    
    cv::Mat img_normal; 
    IR_frame.convertTo(img_normal, CV_32F, 1.0 / 255.0f, 0);
    img_normal = img_normal - cv::Scalar(0.485, 0.456, 0.406);//mean
    img_normal = img_normal / cv::Scalar(0.229, 0.224, 0.225);//std

    //preprocess image - pad    
    cv::Mat img_final_float(640, 640, CV_32FC3);
    cv::resize(img_normal, img_final_float, cv::Size(640, 640));

    //Convert image into input Tensor
    TensorShape input_shape_final = { {1, img_final_float.rows, img_final_float.cols, img_final_float.channels()}, 4 };    // python equivalent: input_shape = (1, 28, 28, 1)
    auto input_values = tf_obj_unique_ptr(img2tensor(img_final_float, input_shape_final));
    if (!input_values) {
        return;
    }
  
    //Run model and get outputs
    CStatus status;
    TF_Tensor* inputs[] = { input_values.get() };
    TF_Tensor* outputs[3] = {};
 
    TF_SessionRun(session->session.get(), nullptr,
        &session->inputs, inputs, 1,
        session->outputs, outputs, 3,
        nullptr, 0, nullptr, status.ptr);
    if (status.failure()) {
        status.dump_error();
        //return -1;
    }
        
    TF_Tensor& box_output = *outputs[0];
    TF_Tensor& scores_output = *outputs[1];
    TF_Tensor& label_output = *outputs[2];
   
    size_t output_size;
    const float* score_output_array;
    const float* box_output_array;
    const int* label_output_array;

    output_size = TF_TensorByteSize(&scores_output) / sizeof(float);
    score_output_array = (const float*)TF_TensorData(&scores_output);
    box_output_array = (const float*)TF_TensorData(&box_output);
    label_output_array = (const int*)TF_TensorData(&label_output);    
    
    float scale = inpHeight / 640.0f;
    //clear boxes, classes ids

    float confThreshold = min(PeopleconfThreshold, HeadconfThreshold);
    NumDetections = 0;
    for (int i = 0; i < output_size; i++)
    {
        if (score_output_array[i] > confThreshold && i < MAX_DETECTED_OBJECTS)
        {
            //scores
            confidences[i] = score_output_array[i];
            //labels
            classIds[i] = label_output_array[i];
            //boxes
            int nXMin = (int)(box_output_array[4 * i + 0] * scale);
            int nYMin = (int)(box_output_array[4 * i + 1] * scale);
            int nXMax = (int)(box_output_array[4 * i + 2] * scale);
            int nYMax = (int)(box_output_array[4 * i + 3] * scale);

            //iou based detection with other objects of same class
            //to remove overlaps
            bool bOverlapExists = false;
            for (int j = 0; j < NumDetections; j++)
            {
                //nms only within same class
                if (classIds[j] == label_output_array[i])
                {
                    //(a). Find the largest(x, y) coordinates for the start of the bounding boxand the smallest(x, y) coordinates
                    float x1 = MAX(boxes[4 * j + 0], (float)nXMin); // For the 0th coordinate
                    float y1 = MAX(boxes[4 * j + 1], (float)nYMin); // For the 1st coordinate
                    float x2 = MIN(boxes[4 * j + 2], (float)nXMax); // For the 2nd coordinate
                    float y2 = MIN(boxes[4 * j + 3], (float)nYMax); // For the 3rd coordinate

                    //(b). Compute the width and height of the bounding box (which will give the intersection of the bounding boxes)
                    float nOverlapW = MAX(0, x2 - x1);
                    float nOverlapH = MAX(0, y2 - y1);

                    //(c). Compute the ratio of overlap as IOU  = Intersection/area1 + area2 - Intersection
                    float nIntersection = nOverlapW * nOverlapH;
                    if (nIntersection < 0.0f)
                    {
                        continue;
                    }

                    float nArea1 = abs((float)(boxes[4 * j + 3] - boxes[4 * j + 1])
                        * (float)(boxes[4 * j + 2] - boxes[4 * j + 0]));
                    float nArea2 = abs((float)(nYMax - nYMin) * (float)(nXMax - nXMin));

                    float nUnion = nArea1 + nArea2 - nIntersection;
                    float fIOU = nIntersection / nUnion;

                    //(d). # if there is sufficient overlap, suppress the current bounding box
                    if (fIOU > IOU_THRESHOLD)
                    {
                        bOverlapExists = true;
                        break;
                    }
                }
            }
            if (bOverlapExists == false)
            {
                //add this
                boxes[4 * i + 0] = nXMin;
                boxes[4 * i + 1] = nYMin;
                boxes[4 * i + 2] = nXMax;
                boxes[4 * i + 3] = nYMax;
                NumDetections++;
            }            
        }
        else
        {
            //these are sorted so we can break from here
            break;
        }
    }  
}

int ObjectDetector::GetDetections(DetectedObjectInfo* oDetections, 
    int DetectionClass, float nConfThreshold)
{
    if (type == DETECTOR_TYPE_NONE) return 0;

    int nDetCount = 0;
    for (size_t idx = 0; idx < NumDetections; ++idx)
    {
        if (classIds[idx] == DetectionClass && confidences[idx] > nConfThreshold)
        {
            oDetections->nId = (short)idx;
            oDetections->nClass = (short)classIds[idx];
            oDetections->nConfidence = (short)(confidences[idx] * 100);
            oDetections->nMinX = (short)boxes[4 * idx + 0];
            oDetections->nMinY = (short)boxes[4 * idx + 1];
            oDetections->nMaxX = (short)boxes[4 * idx + 2];
            oDetections->nMaxY = (short)boxes[4 * idx + 3];
            oDetections++;
            nDetCount++;

            if (nDetCount >= MAX_DETECTED_OBJECTS)
            {
                break;
            }
        }
    }

    return(nDetCount);
}

void ObjectDetector::CloseModule()
{
}


