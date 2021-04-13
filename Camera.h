//
// Created by liuzikai on 3/11/21.
//

#ifndef CAMERA_H
#define CAMERA_H

#include "Common.h"
#include <thread>
// #include <atomic>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>



class Camera {
public:

    ~Camera();

    struct ParameterSet {
        int cameraID = 0;

        int fps = 120;

        bool enableGamma = false;
        double gamma = 0;
    };

    bool open(const SharedParameters &shared, const ParameterSet &params);

    bool isOpened() const { return cap.isOpened(); }

    string getCapInfo() const { return capInfoSS.str(); };

    void release();

    unsigned int getFrameID() const { return bufferFrameID[lastBuffer]; }

    const cv::Mat &getFrame() const { return buffer[lastBuffer]; }

    using NewFrameCallBack = void (*)(void *);

    void registerNewFrameCallBack(NewFrameCallBack callBack, void *param);


private:

    SharedParameters sharedParams;

    cv::VideoCapture cap;
    ParameterSet capParams;

    std::stringstream capInfoSS;

    // Double buffering
    int lastBuffer = 0;
    cv:: Mat buffer[2];
    unsigned int bufferFrameID[2] = {0, 0};

    std::thread *th = nullptr;
    bool threadShouldExit = false;

    std::vector<std::pair<NewFrameCallBack, void *>> callbacks;

    void readFrameFromCamera();

};



#endif //CAMERA_H