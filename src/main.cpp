//
// Created by liuzikai on 4/5/21.
//
#include "Vision.h"
#include "Camera.h"
#include "Control.h"


bool visionNeedsHandling=false;
vector<cv::Point> newDevices;
vector<cv::Point> removedDevices;
using namespace cv;
using namespace std;
// static pthread_t vision_thread;

std::unique_ptr<Control> control;

int main(int argc, char** argv){

    control = std::make_unique<Control>();

    VideoCapture cap;
    // open the default camera using default API
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    cap.open(deviceID, apiID);
    // set the resolution of camera
    cap.set(CAP_PROP_FRAME_WIDTH, sharedParams.imageWidth);
    cap.set(CAP_PROP_FRAME_HEIGHT, sharedParams.imageHeight);
    cout << "frame width is " << cap.get(CAP_PROP_FRAME_WIDTH) <<
         " frame height is " << cap.get(CAP_PROP_FRAME_HEIGHT) << endl;
    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    //--- GRAB AND WRITE LOOP -----
    cout << "Start streaming the video" << endl;
    DeviceManager devices_manager_interface;
    Vision my_vision;
    for (;;) {
        Mat frame;
        Mat frameCalibration;
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);  // or: cap >> frame;
        // check if we succeeded
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        image_locations = my_vision.processing(frameCalibration);
        vector<Point> image_locations;
        //        cout<<"before launch test"<<endl;
        image_locations = my_vision.processing(frame);
        // cout<<"detected positions"<<endl;
        // for (auto &point: image_locations) {
        //     cout << "(" << point.x << "," << point.y << ")" << endl;
        // }
        vector<cv::Point> inserted;
        vector<cv::Point> deleted;

        devices_manager_interface.updateLocationMapping(image_locations, inserted, deleted);

        vector<cv::Point> inserted_real=devices_manager_interface.getRealLocation(inserted,sharedParams.imageWidth,sharedParams.imageHeight);
        vector<cv::Point> deleted_real=devices_manager_interface.getRealLocation(deleted,sharedParams.imageWidth,sharedParams.imageHeight);
        if (inserted.size()!=0){
            cout<<"inserted positions"<<endl;
            for(auto& point:inserted){
                cout <<" position in image "<<"(" << point.x << "," << point.y << ")" << endl;
            }
            for(auto& point:inserted_real){
                cout <<" real position in the table "<<"(" << point.x << "," << point.y << ")" << endl;
            }
        }
        if (deleted.size()!=0) {
            cout << "removed positions" << endl;
            for (auto &point:deleted) {
                cout <<" position in image "<< "(" << point.x << "," << point.y << ")" << endl;
            }
            for(auto& point:deleted_real){
                cout <<" real position in the table "<<"(" << point.x << "," << point.y << ")" << endl;
            }
        }
        // blocking of the vision handling
        // while true, wait
        while (visionNeedsHandling){
//            if((inserted_real.size() || deleted_real.size())){
//                newDevices = inserted_real;
//                removedDevices = deleted_real;
//                visionNeedsHandling = true;
//                break;
//            }
        }
        if (!visionNeedsHandling && (inserted_real.size() || deleted_real.size())){
            newDevices = inserted_real;
            removedDevices = deleted_real;
            visionNeedsHandling = true;
        }

    }
    return 0;
}
