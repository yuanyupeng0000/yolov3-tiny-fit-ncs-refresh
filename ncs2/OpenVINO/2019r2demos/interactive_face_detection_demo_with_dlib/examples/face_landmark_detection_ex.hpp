#ifndef FACE_LANDMARK_DETECTION_EX_HPP
#define FACE_LANDMARK_DETECTION_EX_HPP
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing.h>
#include <samples/ocv_common.hpp>
#include <iostream>
std::vector<float> main_(const dlib::shape_predictor& sp, const dlib::frontal_face_detector& ffd,
          const cv::Mat& img, const cv::Rect &face_rect);
#endif // FACE_LANDMARK_DETECTION_EX_HPP
