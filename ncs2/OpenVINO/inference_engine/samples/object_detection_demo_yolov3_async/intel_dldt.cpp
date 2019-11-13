#include "intel_dldt.h"
#include "detector.h"
#include <memory>
#include <iostream>
#include <stdio.h>
#include <pthread.h>
#include <string>
#include <fstream>
#include <sstream>

#define NCS_NUM 50
#define DETECTOR_NUM 1
#define DETECTOR_MODE_ONE
Detector* detectors[DETECTOR_NUM] = {0};

// Intel deep learning development tool parameter
struct IntelDldtParam {
    std::string model_xml, model_bin, model_dev;
    float thresh_confidence, thresh_iou;
    int number_infer_requests;

    IntelDldtParam(std::string model_xml="", std::string model_bin="", std::string model_dev="",
                   float thresh_confidence=0.3, float thresh_iou=0.5, int number_infer_requests=4) {
        this->model_xml = model_xml;
        this->model_bin = model_bin;
        this->model_dev = model_dev;
        this->thresh_confidence = thresh_confidence;
        this->thresh_iou = thresh_iou;
        this->number_infer_requests = number_infer_requests;
    }
};

static bool intel_dldt_query_str(const std::string& src, const std::string& query, std::string& target){
    size_t pos_start = src.find(query, 0);
    if(pos_start == std::string::npos){
        std::cout << "[ ERRO ] " << query << " not find ." << std::endl;
        return false;
    }
    else{
        size_t pos_end = src.find("\n", pos_start + query.size());
        if(pos_end == std::string::npos){
            std::cout << "[ ERRO ] " << query << " not find ." << std::endl;
            return false;
        }
        else{
            std::string value(src.substr(pos_start + query.size(), pos_end-pos_start-query.size()));
            if(value.empty()){
                std::cout << "[ ERRO ] " << query << " not find ." << std::endl;
                return false;
            }
            std::cout << "[ INFO ] " << query << value << std::endl;
            target = value;
            return true;
        }
    }
}
static void intel_dldt_read_config(const std::string& config, IntelDldtParam& intel_dldt_param){
    std::ifstream cfg(config);
    std::stringstream buffer;
    buffer << cfg.rdbuf();
    std::string contents(buffer.str());
    std::string target;
    if(intel_dldt_query_str(contents, "model_xml=", target)){
        intel_dldt_param.model_xml = target;
    }
    if(intel_dldt_query_str(contents, "model_bin=", target)){
        intel_dldt_param.model_bin = target;
    }
    if(intel_dldt_query_str(contents, "model_dev=", target)){
        intel_dldt_param.model_dev = target;
    }
    if(intel_dldt_query_str(contents, "thresh=", target)){
        intel_dldt_param.thresh_confidence = std::stof(target);
    }
    if(intel_dldt_query_str(contents, "nms=", target)){
        intel_dldt_param.thresh_iou = std::stof(target);
    }
    if(intel_dldt_query_str(contents, "nireq=", target)){
        intel_dldt_param.number_infer_requests = std::stoi(target);
    }
}

int intel_dldt_init(const std::string& config/*const IntelDldtParam& intel_dldt_param*/){
    IntelDldtParam intel_dldt_param;
    intel_dldt_read_config(config, intel_dldt_param);
    for(int i=0; i<DETECTOR_NUM; i++){
        detectors[i] = new Detector(intel_dldt_param.model_xml, intel_dldt_param.model_bin,
                                    intel_dldt_param.model_dev, intel_dldt_param.thresh_confidence,
                                    intel_dldt_param.thresh_iou, intel_dldt_param.number_infer_requests);
    }
    return NCS_NUM;
}
void intel_dldt_detect(const cv::Mat frame, int NCS_ID, std::vector<DetectionObject>& objs){
    std::cout << "[ INFO ] NCS_ID " << NCS_ID << std::endl;
#ifdef DETECTOR_MODE_ONE
    detectors[0]->Detect(NCS_ID, frame, objs);
#else
    detectors[NCS_ID]->Detect(frame, objs);
#endif
}
