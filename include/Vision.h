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

#include <iostream>
#include <vector>
#include <stdio.h>
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <sys/fcntl.h>


using namespace cv;
using namespace std;

class Vision {
public:
    // member variable
    // member function
    vector <Point> processing(Mat &frame);
    void drawRotatedRect(Mat &img, const RotatedRect &rect, const Scalar &boarderColor);
    vector <RotatedRect> findBoundingBox(const Mat &image_BrightnessThreshold, Mat &drawing);
    void gammaCorrection(const Mat &img, Mat& gamma_corrected, const double gamma_);




    // member variable 
    int blur_kernel_size_ = 9;
    int min_contour_area_ = 1000;
    int max_contour_area_ = 10000000;
    // extend is the ratio between the contour area and the bounding box area
    float extend_threshold_ = 0.8;
    // smaller threshold means the pixel need to be dark enough to be set to light (so to be detected)
    int black_value_pick_up_ = 50;
    // bigger threshold means the pixel need to be bright enough to be set to light (so to be detected)
    int white_value_pick_up_ = 180;
    float gamma_val_darker_ = 1.7;
    float aspectRatio_max_ =2.5;
    float aspectRatio_min_ =1.5;
};

#endif //ECE445_TEAM24_VISION_H
