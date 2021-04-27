//
// Created by liuzikai on 4/5/21.
//
#ifndef ECE445_TEAM24_VISION_H
#define ECE445_TEAM24_VISION_H

#include "Common.h"
#include <thread>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect.hpp>


class Vision {
public:

    Vision();

    void fetchDeviceDiff(vector<cv::RotatedRect> &newDevices, vector<cv::RotatedRect> &deletedDevices);

    std::vector<cv::RotatedRect> process(cv::Mat &frame);

    void imageCalibrate(const cv::Mat& frame, cv::Mat& frameCalibrated);

private:

    std::thread *th = nullptr;
    void runVisionThread();

    // -------------------------------- Video Capture --------------------------------

    cv::VideoCapture cap;

    static constexpr int CAMERA_FRAME_WIDTH = 1280;
    static constexpr int CAMERA_FRAME_HEIGHT = 720;

    static constexpr int TABLE_WIDTH = 360;
    static constexpr int TABLE_HEIGHT = 260;

    // -------------------------------- Image processing --------------------------------

    

    void drawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &boarderColor);

    std::vector<cv::RotatedRect> findBoundingBoxes(const cv::Mat &brightnessThreshold,
                                                   std::vector<std::vector<cv::Point>> &contours);

    void gammaCorrect(const cv::Mat &img, cv::Mat &gammaCorrected, double gamma);

    void drawBoundingBoxes(std::vector<cv::RotatedRect> &boundingBoxes, cv::Mat &drawing, cv::Mat &frame);

    std::vector<std::vector<cv::Point>> findAndDrawContours(const cv::Mat &brightnessThreshold, cv::Mat &drawing);

    


    // member variable 
    int blur_kernel_size_ = 9;
    int min_contour_area_ = 25000;
    int max_contour_area_ = 10000000;
    // extend is the ratio between the contour area and the bounding box area
    float extend_threshold_ = 0.75;
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

    // -------------------------------- Device Management --------------------------------

    void updateDevices(const vector<cv::RotatedRect> &boxes);

    long long nextUID = 0;

    struct Device {
        cv::RotatedRect rect;
        int counter = 0;
        bool reported = false;
    };

    map<long long, Device> existingDevices;

    bool rectMatched(const cv::RotatedRect &cur, const cv::RotatedRect &ref);

    bool pointClose(const cv::Point &cur, const cv::Point &ref);

    cv::RotatedRect getRealRect(cv::RotatedRect& old_rect);

    static constexpr int COORDINATE_OFFSET_THRESHOLD = 15;
    static constexpr float INTERSECTION_AREA_THRESHOLD = 0.9;
    static constexpr float AREA_RATIO_THRESHOLD = 0.9;
    static constexpr float ROTATED_RECT_UPDATE_RATE = 0.5;
    static constexpr int COUNTER_THRESHOLD = 30;

    cv::RotatedRect combineRect(const cv::RotatedRect &cur, const cv::RotatedRect &ref);

};

#endif //ECE445_TEAM24_VISION_H
