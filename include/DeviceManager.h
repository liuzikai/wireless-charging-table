//
// Created by liuzikai on 4/5/21.
//
#ifndef DEVICE_MANAGER_H
#define DEVICE_MANAGER_H

#include "Common.h"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>

class DeviceManager {
public:

    bool withinMargin(const cv::Point &curPoint, const cv::Point &refPoint);

    void updateLocationMapping(const vector <cv::Point> &locations);

    void sendingLocation(const vector <cv::Point> &locations);

    vector <cv::Point> getRealLocation(const vector <cv::RotatedRect> &boundingBox, int imageWidth, int imageHeight);

private:

    // member variable
    vector <std::pair<cv::Point, int>> locationMap;
};

#endif //DEVICE_MANAGER_H