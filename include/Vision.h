//
// Created by liuzikai on 4/5/21.
//
#ifndef ECE445_TEAM24_VISION_H
#define ECE445_TEAM24_VISION_H


#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <iostream>
#include <vector>
#include <stdio.h>
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <sys/fcntl.h>

#include "Camera.h"
#include "Common.h"
#include "DeviceManager.h"

extern SharedParameters sharedParams;
extern Camera::ParameterSet cameraParams;

class Vision {
public:
    std::vector<cv::Point> processing(cv::Mat &frame);

    void drawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &boarderColor);

    std::vector<cv::RotatedRect> findBoundingBox(const cv::Mat &image_BrightnessThreshold,
                                                 std::vector<std::vector<cv::Point>> &contours);

    void gammaCorrection(const cv::Mat &img, cv::Mat &gamma_corrected, double gamma_);

    void draw_bounding_box(std::vector<cv::RotatedRect> &BoundingBox, cv::Mat &drawing, cv::Mat &frame);

    std::vector<std::vector<cv::Point>> find_draw_contours(const cv::Mat &image_BrightnessThreshold, cv::Mat &drawing);

    // member variable 
    int blur_kernel_size_ = 9;
    int min_contour_area_ = 2500;
    int max_contour_area_ = 10000000;
    // extend is the ratio between the contour area and the bounding box area
    float extend_threshold_ = 0.8;
    // smaller threshold means the pixel need to be dark enough to be set to light (so to be detected)
    int black_value_pick_up_ = 30;
    // bigger threshold means the pixel need to be bright enough to be set to light (so to be detected)
    int white_value_pick_up_ = 180;
    float gamma_val_darker_ = 1.9;
    float gamma_val_hsv_ = 2.5;
    float aspectRatio_max_ = 2.5;
    float aspectRatio_min_ = 1.5;
    // hue range is [0,179], saturation range is [0,255], and value range is [0,255]
    // H between 0 to 360
    float high_H_ = 45;
    float low_H_ = 17;
    // S between 0 to 1
    float high_S_ = 160;
    float low_S_ = 20;
    // V between 0 to 1
    float high_V_ = 150;
    float low_V_ = 40;
};

#endif //ECE445_TEAM24_VISION_H
