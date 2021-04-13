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
    vector <RotatedRect> find_bounding_box(Mat &image_BrightnessThreshold, Mat &drawing);
};

#endif //ECE445_TEAM24_VISION_H
