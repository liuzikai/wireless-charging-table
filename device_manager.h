//
// Created by liuzikai on 4/5/21.
//
#ifndef DEVICE_MANAGER_H
#define DEVICE_MANAGER_H

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>

#include <iostream>
#include <vector>
#include <map>
// #include <pair>
// #include <Point>
#include <stdio.h>
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <sys/fcntl.h>

using namespace cv;
using namespace std;

class Device_manager {
public:
    // member variable
    vector<std::Pair<Point,int>> location_map;
    bool within_margin(const Point& cur_point, const Point& ref_point);
    void update_location_mapping(vector <Point> locations);
    void sending_location(vector <Point> locations);
    vector <Point> get_real_location(vector <RotatedRect> &BoundingBox, int image_width, int image_height);
};

#endif //DEVICE_MANAGER_H
