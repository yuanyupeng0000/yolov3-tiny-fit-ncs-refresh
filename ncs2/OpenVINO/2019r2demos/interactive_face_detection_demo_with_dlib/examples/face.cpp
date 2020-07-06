// Copyright (C) 2018-2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

#include <string>
#include <map>
#include <utility>
#include <list>
#include <vector>

#include "face.hpp"

Face::Face(size_t id, cv::Rect& location):
    _location(location), _intensity_mean(0.f), _id(id), _age(-1),
    _maleScore(0), _femaleScore(0), _headPose({0.f, 0.f, 0.f}),
    _isAgeGenderEnabled(false), _isEmotionsEnabled(false), _isHeadPoseEnabled(false), _isLandmarksEnabled(false) {
}

void Face::updateAge(float value) {
    _age = (_age == -1) ? value : 0.95f * _age + 0.05f * value;
}

void Face::updateGender(float value) {
    if (value < 0)
        return;

    if (value > 0.5) {
        _maleScore += value - 0.5f;
    } else {
        _femaleScore += 0.5f - value;
    }
}

void Face::updateEmotions(std::map<std::string, float> values) {
    for (auto& kv : values) {
        if (_emotions.find(kv.first) == _emotions.end()) {
            _emotions[kv.first] = kv.second;
        } else {
            _emotions[kv.first] = 0.9f * _emotions[kv.first] + 0.1f * kv.second;
        }
    }
}

void Face::updateEyeState(float thresh){
    //36~47 is the eye points
    float l_eye_h, r_eye_h, l_eye_w, r_eye_w;
    l_eye_h = _landmarks[41*2+1] - _landmarks[37*2+1]
            + _landmarks[40*2+1] - _landmarks[38*2+1];

    r_eye_h = _landmarks[47*2+1] - _landmarks[43*2+1]
            + _landmarks[46*2+1] - _landmarks[44*2+1];

    l_eye_w = _landmarks[39*2] - _landmarks[36*2];
    r_eye_h = _landmarks[45*2] - _landmarks[42*2];

    float eye_aspect_ratio = (l_eye_h + r_eye_h)/(l_eye_w + r_eye_w);
    std::cout  << "esr:" << eye_aspect_ratio << std::endl;
    if(eye_aspect_ratio < thresh){
        _isEyeClosed = true;
        _eyeKeepClosedFrameCount += 1;
    }
    else{
        _isEyeClosed = false;
        _eyeKeepClosedFrameCount = 0;
    }

}

void Face::updateHeadPostureState(HeadPoseDetection::Results& values){
    float diff_yaw, diff_pitch, diff_roll;
    diff_pitch = values.angle_p - _headPose.angle_p;
    diff_roll = values.angle_r - _headPose.angle_r;
    diff_yaw = values.angle_y - _headPose.angle_y;

    if (diff_pitch  > 0){
        _headKeepDownFrameCount += 1;
        _headKeepUpFrameCount = 0;
    }
    else
    {
        _headKeepUpFrameCount += 1;
        _headKeepDownFrameCount = 0;
    }
    if (diff_roll > 0){
        _headKeepLeftFrameCount += 1;
        _headKeepRightFrameCount = 0;
    }
    else
    {
        _headKeepRightFrameCount += 1;
        _headKeepLeftFrameCount = 0;
    }
    if (diff_yaw > 0){
        _headKeepTurnLeftFrameCount +=1;
        _headKeepTurnRightFrameCount = 0;
    }
    else
    {
        _headKeepTurnRightFrameCount +=1;
        _headKeepTurnLeftFrameCount = 0;
    }
}

void Face::updateLipState(float thresh){
    //60~67 is the lips inner side points
    float m_h, m_w;
    m_h = (_landmarks[58*2+1] - _landmarks[50*2+1]
            + _landmarks[57*2+1] - _landmarks[51*2+1]
            + _landmarks[56*2+1] - _landmarks[52*2+1])/3.0;

    m_w = (_landmarks[64*2] + _landmarks[54*2])/2.0- (_landmarks[48*28]
             + _landmarks[60*2])/2.0;

    float mouse_aspect_ratio = m_h/m_w;
    printf("\nmouse_aspect_ratio=%f\n", mouse_aspect_ratio);
    std::cout  << "msr:" << mouse_aspect_ratio << std::endl;
    if(mouse_aspect_ratio > thresh){
        _isMouseOpened = true;
        _mouseKeepOpenedFrameCount += 1;
    }
    else{
        _isMouseOpened = false;
        _mouseKeepOpenedFrameCount = 0;
    }

}

void Face::updateHeadPose(HeadPoseDetection::Results values) {
    updateHeadPostureState(values); //yyp added
    _headPose = values;
}

void Face::updateLandmarks(std::vector<float> values) {
    _landmarks = std::move(values);
    updateEyeState();
    updateLipState();
}

int Face::getAge() {
    return static_cast<int>(std::floor(_age + 0.5f));
}

bool Face::getEyeState(){
    return _isEyeClosed;
}
bool Face::getSleepyState(){
    if(_eyeKeepClosedFrameCount > 15 || this->getMouseOpenedState()){
        return true;
    }
    return false;
}
bool Face::getMouseOpenedState(){
    if(_mouseKeepOpenedFrameCount > 5){
        return true;
    }
    return false;
}
bool Face::getHeadPostureString(std::string& posture){
    posture.clear();
    if (_headPose.angle_r < -10 ){
        posture += "lean-right ";
    }
    else if (_headPose.angle_r > 10){
        posture += "lean-left ";
    }
    if (20 < _headPose.angle_p ){
        posture += "lean-forward ";
    }
    else if (_headPose.angle_p < -10){
        posture += "lean-backward ";
    }

    if (_headPose.angle_y < -10){
        posture += "turn-right ";
    }
    else if (_headPose.angle_y > 10){
        posture += "turn-left ";
    }

    if (posture.empty()){
        posture += "Head pose: common";
    }
    else{
        posture = "Head pose: " + posture;
    }


}

bool Face::isMale() {
    return _maleScore > _femaleScore;
}

std::map<std::string, float> Face::getEmotions() {
    return _emotions;
}

std::pair<std::string, float> Face::getMainEmotion() {
    auto x = std::max_element(_emotions.begin(), _emotions.end(),
        [](const std::pair<std::string, float>& p1, const std::pair<std::string, float>& p2) {
            return p1.second < p2.second; });

    return std::make_pair(x->first, x->second);
}

HeadPoseDetection::Results Face::getHeadPose() {
    return _headPose;
}

const std::vector<float>& Face::getLandmarks() {
    return _landmarks;
}

size_t Face::getId() {
    return _id;
}

void Face::ageGenderEnable(bool value) {
    _isAgeGenderEnabled = value;
}
void Face::emotionsEnable(bool value) {
    _isEmotionsEnabled = value;
}
void Face::headPoseEnable(bool value) {
    _isHeadPoseEnabled = value;
}
void Face::landmarksEnable(bool value) {
    _isLandmarksEnabled = value;
}

bool Face::isAgeGenderEnabled() {
    return _isAgeGenderEnabled;
}
bool Face::isEmotionsEnabled() {
    return _isEmotionsEnabled;
}
bool Face::isHeadPoseEnabled() {
    return _isHeadPoseEnabled;
}
bool Face::isLandmarksEnabled() {
    return _isLandmarksEnabled;
}

float calcIoU(cv::Rect& src, cv::Rect& dst) {
    cv::Rect i = src & dst;
    cv::Rect u = src | dst;

    return static_cast<float>(i.area()) / static_cast<float>(u.area());
}

float calcMean(const cv::Mat& src) {
    cv::Mat tmp;
    cv::cvtColor(src, tmp, cv::COLOR_BGR2GRAY);
    cv::Scalar mean = cv::mean(tmp);

    return static_cast<float>(mean[0]);
}

Face::Ptr matchFace(cv::Rect rect, std::list<Face::Ptr>& faces) {
    Face::Ptr face(nullptr);
    float maxIoU = 0.55f;
    for (auto&& f : faces) {
        float iou = calcIoU(rect, f->_location);
        if (iou > maxIoU) {
            face = f;
            maxIoU = iou;
        }
    }

    return face;
}
