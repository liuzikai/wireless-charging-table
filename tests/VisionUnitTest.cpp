#include "Vision.h"
using namespace std;
using namespace cv;
int main(int argc, char **argv) {
   // open the file for data transmission

#if CAMERA
//     Camera my_camera;
//     my_camera.open(sharedParams, cameraParams);
   //--- INITIALIZE VIDEOCAPTURE ------
   VideoCapture cap;
   // open the default camera using default API
   // cap.open(0);
   // OR advance usage: select any API backend
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
   // !!!!!!! Declaration of this variable must be the outside of the for loop
   // otherwise the location map will be overwrite every time
//    cout<<"before entering for loop"<<endl;
   DeviceManager devices_manager_interface;
   Vision my_vision;
   for (;;) {
       Mat frame;
       // wait for a new frame from camera and store it into 'frame'
       cap.read(frame);  // or: cap >> frame;
       // check if we succeeded
       if (frame.empty()) {
           cerr << "ERROR! blank frame grabbed\n";
           break;
       }

#if SHOW_ORIG_IMAGE
       // show live and wait for a key with timeout long enough to show images
       imshow("Live", frame);
       // wait for a key with 5 miliseconds
       if (waitKey(5) >= 0)
           break;
#endif
       // store the image
//         Mat frame=my_camera.getFrame();
       imwrite("test.jpg", frame);
       // break;
       // call a function to process the image, passing by reference

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
       // cout<<"inserted positions"<<endl;
       // for(auto& point:inserted){

       //     cout << "(" << point.x << "," << point.y << ")" << endl;
       // }
       // cout<<"removed positions"<<endl;
       // for(auto& point:deleted){

       //     cout << "(" << point.x << "," << point.y << ")" << endl;
       // }
       if (!visionNeedsHandling && (inserted.size() || deleted.size())){
           newDevices = inserted;
           removedDevices = deleted;
           visionNeedsHandling = true;
       }

   }
#endif

#if !CAMERA
   // read from the cmd arg
   if( argc != 15) {
       cout <<" Usage: ./VisionUnitTest image_to_process min_contour_area max_contour_area extend black_value_pick_up white_value_pick_up gamma_val_darker gamma_val_brighter high_H_ low_H_ high_S_ low_S_ high_V_ low_V_ " << endl;
       return -1;
   }
   Mat frame=imread(argv[1]);
   if (frame.empty()) {
       cerr << "ERROR! blank frame grabbed\n";
   }
   // imshow("Live", frame);
   Vision my_vision;
   // some extra code to help fine-tune the parameter
   my_vision.min_contour_area_=atoi(argv[2]);
   my_vision.max_contour_area_=atoi(argv[3]);
   my_vision.extend_threshold_ = atof(argv[4]);
   my_vision.black_value_pick_up_=atoi(argv[5]);
   my_vision.white_value_pick_up_=atoi(argv[6]);
   my_vision.gamma_val_darker_=atof(argv[7]);
   my_vision.gamma_val_hsv_=atof(argv[8]);
   // cout<<"checking value "<<min_contour_area<<" "<<max_contour_area<<" "<<extend_threshold<<" "<<black_value_pick_up<<" "<<gamma_val_darker<<endl;
   my_vision.high_H_ = atof(argv[9]);
   my_vision.low_H_ = atof(argv[10]);
   my_vision.high_S_ = atof(argv[11]);
   my_vision.low_S_ = atof(argv[12]);
   my_vision.high_V_ = atof(argv[13]);
   my_vision.low_V_ = atof(argv[14]);


   my_vision.processing(frame);
#endif
   // close the file
   // the camera will be deinitialized automatically in VideoCapture destructor
   return 0;
}