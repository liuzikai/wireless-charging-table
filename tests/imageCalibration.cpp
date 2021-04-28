#include "opencv2/opencv.hpp"
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char **argv){
    VideoCapture inputVideo;
    cout<<"test"<<endl;
    // open the default camera using default API
    // cap.open(0);
    // OR advance usage: select any API backend
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    inputVideo.open(deviceID, apiID);

    inputVideo.set(CAP_PROP_FRAME_WIDTH, 1280);
    inputVideo.set(CAP_PROP_FRAME_HEIGHT, 720);
    cout << "frame width is " << inputVideo.get(CAP_PROP_FRAME_WIDTH) <<
         " frame height is " << inputVideo.get(CAP_PROP_FRAME_HEIGHT) << endl;
    //inputVideo.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    //inputVideo.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    if (!inputVideo.isOpened())
    {
        cout << "Could not open the input video " << endl;
        return -1;
    }
    Mat frame;
    Mat frameCalibrated;
    string imgname;
    int f = 1;
    while (1) //Show the image captured in the window and repeat
    {
        inputVideo >> frame;              // read
        if (frame.empty()) break;         // check if at end
        imshow("Camera", frame);
        char key = waitKey(1);
        if (key == 27)break;
        if (key == 'q' || key == 'Q')
        {
            imgname = to_string(f++) + ".jpg";
            imwrite(imgname, frame);
        }
    }
    cout << "Finished writing" << endl;
    return 0;
}
