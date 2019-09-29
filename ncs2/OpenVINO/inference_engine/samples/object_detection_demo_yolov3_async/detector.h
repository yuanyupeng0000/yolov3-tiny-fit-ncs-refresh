#ifndef DETECTOR_H
#define DETECTOR_H

#include <stdio.h>
#include <string>
#include <iostream>
#include <samples/ocv_common.hpp>
#include <inference_engine.hpp>
#include "Common.h"
using namespace InferenceEngine;
#define MAX_INFER_SIZE 96

class Detector
{
public:
    Detector(const std::string& inputXml,
             const std::string& inputBin,
             const std::string& inputDevice,
             const float thresh=0.3,
             const float iou=0.7,
             const int nireq=2);
    void Detect(const cv::Mat frame);
    int Detect(const cv::Mat frame, std::vector<DetectionObject>& objects);
    int Detect(int idx, const cv::Mat frame, std::vector<DetectionObject>& objects);
    void InitIdxSteps();
    int GetInferIndexes(int index);
    InferRequest::Ptr async_infer_request_next;
    InferRequest::Ptr async_infer_request_curr;
    CNNNetReader netReader;
    InputsDataMap inputInfo;
    OutputsDataMap outputInfo;
    std::vector<std::string> labels;
    InferRequest::Ptr IfReqs[MAX_INFER_SIZE];
    float thresh;
    int nireq;
    int current_request_id;
    float iou;
    std::map<int, int> idx_steps;
    std::string input_xml;

    /**
     * @brief A destructor
     */
    virtual ~Detector() {}
};

#endif // DETECTOR_H
