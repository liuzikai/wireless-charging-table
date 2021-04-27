#include "Vision.h"
using namespace std;
using namespace cv;
#define CAMERA 1

std::unique_ptr<Vision> vision;
int main(int argc, char **argv) {
   // open the file for data transmission

#if CAMERA
    vision = std::make_unique<Vision>();
    while (1);
#endif

#if !CAMERA
   // read from the cmd arg
   if( argc != 2) {
       cout <<" Usage: ./VisionUnitTest image_to_process min_contour_area max_contour_area extend black_value_pick_up white_value_pick_up gamma_val_darker gamma_val_brighter high_H_ low_H_ high_S_ low_S_ high_V_ low_V_ " << endl;
       return -1;
   }
   Mat frame=imread(argv[1]);
   if (frame.empty()) {
       cerr << "ERROR! blank frame grabbed\n";
   }
   // imshow("Live", frame);

    Vision my_vision;
    Mat frameCalibration;
    my_vision.imageCalibrate(frame, frameCalibration);
//   // some extra code to help fine-tune the parameter
//   my_vision.min_contour_area_=atoi(argv[2]);
//   my_vision.max_contour_area_=atoi(argv[3]);
//   my_vision.extend_threshold_ = atof(argv[4]);
//   my_vision.black_value_pick_up_=atoi(argv[5]);
//   my_vision.white_value_pick_up_=atoi(argv[6]);
//   my_vision.gamma_val_darker_=atof(argv[7]);
//   my_vision.gamma_val_hsv_=atof(argv[8]);
//   // cout<<"checking value "<<min_contour_area<<" "<<max_contour_area<<" "<<extend_threshold<<" "<<black_value_pick_up<<" "<<gamma_val_darker<<endl;
//   my_vision.high_H_ = atof(argv[9]);
//   my_vision.low_H_ = atof(argv[10]);
//   my_vision.high_S_ = atof(argv[11]);
//   my_vision.low_S_ = atof(argv[12]);
//   my_vision.high_V_ = atof(argv[13]);
//   my_vision.low_V_ = atof(argv[14]);
    Mat frameCalibratedCropped = frameCalibration(
                {40, 680},
                {110, 1170});
    cout<<"reach here!"<<endl;
    my_vision.process(frameCalibratedCropped);
#endif
   // close the file
   // the camera will be deinitialized automatically in VideoCapture destructor
   return 0;
}