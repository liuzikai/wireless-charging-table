#include "Vision.h"
#include "Camera.h"
#include "Common.h"
#include "DeviceManager.h"
#include <iostream>

#define GPU 0

#if GPU
#include <opencv2/gpu/gpu.hpp>  
#endif

// solution1: adjust the parameter ( by passing the parameter to the program )
// solution2: change to DNN
// solution3: subtract two images and find the difference

// problem1: bounding box for small object & big object
// problem2: aspect ratio change
// problem3: rect size for small object & big object
// problem4: white and black thresholding value ---> sharp change of the boundary of the object

// other image pre-processing technique beside the black & white thresholding

using namespace cv;
using namespace std;
// height and width in millimeter

// flag for show image 
#define SHOW_ORIG_IMAGE 0
#define SHOW_GRAY_IMAGE 0
#define SHOW_THRESHOLD_IMAGE 0
#define SHOW_CONTOUR_IMAGE 0
#define SHOW_ANNOTED_IMAGE 1
#define CAMERA 0

SharedParameters sharedParams;
Camera::ParameterSet cameraParams;

// for random color
RNG rng(12345);

// calibrate the image 
// https://docs.opencv.org/master/d6/d55/tutorial_table_of_content_calib3d.html
// https://docs.opencv.org/master/d4/d94/tutorial_camera_calibration.html

// int main(int argc, char **argv) {
//     // open the file for data transmission

// #if CAMERA
// //     Camera my_camera;
// //     my_camera.open(sharedParams, cameraParams);
//     //--- INITIALIZE VIDEOCAPTURE ------
//     VideoCapture cap;
//     // open the default camera using default API
//     // cap.open(0);
//     // OR advance usage: select any API backend
//     int deviceID = 0;             // 0 = open default camera
//     int apiID = cv::CAP_ANY;      // 0 = autodetect default API
//     // open selected camera using selected API
//     cap.open(deviceID, apiID);
//     // set the resolution of camera
//     cap.set(CAP_PROP_FRAME_WIDTH, sharedParams.imageWidth);
//     cap.set(CAP_PROP_FRAME_HEIGHT, sharedParams.imageHeight);
//     cout << "frame width is " << cap.get(CAP_PROP_FRAME_WIDTH) <<
//          " frame height is " << cap.get(CAP_PROP_FRAME_HEIGHT) << endl;
//     // check if we succeeded
//     if (!cap.isOpened()) {
//         cerr << "ERROR! Unable to open camera\n";
//         return -1;
//     }
//     //--- GRAB AND WRITE LOOP ----- 
//     cout << "Start streaming the video" << endl;
//     // !!!!!!! Declaration of this variable must be the outside of the for loop
//     // otherwise the location map will be overwrite every time
// //    cout<<"before entering for loop"<<endl;
//     DeviceManager devices_manager_interface;
//     Vision my_vision;
//     for (;;) {
//         Mat frame;
//         // wait for a new frame from camera and store it into 'frame'
//         cap.read(frame);  // or: cap >> frame;
//         // check if we succeeded
//         if (frame.empty()) {
//             cerr << "ERROR! blank frame grabbed\n";
//             break;
//         }

// #if SHOW_ORIG_IMAGE
//         // show live and wait for a key with timeout long enough to show images
//         imshow("Live", frame);
//         // wait for a key with 5 miliseconds
//         if (waitKey(5) >= 0)
//             break;
// #endif
//         // store the image  
// //         Mat frame=my_camera.getFrame();
//         imwrite("test.jpg", frame);
//         // break;
//         // call a function to process the image, passing by reference

//         vector<Point> image_locations;
// //        cout<<"before launch test"<<endl;
//         image_locations = my_vision.processing(frame);
//         // cout<<"detected positions"<<endl;
//         // for (auto &point: image_locations) {
//         //     cout << "(" << point.x << "," << point.y << ")" << endl;
//         // }
//         vector<cv::Point> inserted;
//         vector<cv::Point> deleted;

//         devices_manager_interface.updateLocationMapping(image_locations, inserted, deleted);
//         // cout<<"inserted positions"<<endl;
//         // for(auto& point:inserted){

//         //     cout << "(" << point.x << "," << point.y << ")" << endl;
//         // }
//         // cout<<"removed positions"<<endl;
//         // for(auto& point:deleted){

//         //     cout << "(" << point.x << "," << point.y << ")" << endl;
//         // }


//     }
// #endif

// #if !CAMERA
//     // read from the cmd arg
//     if( argc != 15) {
//         cout <<" Usage: ./VisionUnitTest image_to_process min_contour_area max_contour_area extend black_value_pick_up white_value_pick_up gamma_val_darker gamma_val_brighter high_H_ low_H_ high_S_ low_S_ high_V_ low_V_ " << endl;
//         return -1;
//     }
//     Mat frame=imread(argv[1]); 
//     if (frame.empty()) {
//         cerr << "ERROR! blank frame grabbed\n";
//     }
//     // imshow("Live", frame);
//     Vision my_vision;
//     // some extra code to help fine-tune the parameter
//     my_vision.min_contour_area_=atoi(argv[2]);
//     my_vision.max_contour_area_=atoi(argv[3]);
//     my_vision.extend_threshold_ = atof(argv[4]);
//     my_vision.black_value_pick_up_=atoi(argv[5]);
//     my_vision.white_value_pick_up_=atoi(argv[6]);
//     my_vision.gamma_val_darker_=atof(argv[7]);
//     my_vision.gamma_val_hsv_=atof(argv[8]);
//     // cout<<"checking value "<<min_contour_area<<" "<<max_contour_area<<" "<<extend_threshold<<" "<<black_value_pick_up<<" "<<gamma_val_darker<<endl;
//     my_vision.high_H_ = atof(argv[9]);
//     my_vision.low_H_ = atof(argv[10]);
//     my_vision.high_S_ = atof(argv[11]);
//     my_vision.low_S_ = atof(argv[12]);
//     my_vision.high_V_ = atof(argv[13]);
//     my_vision.low_V_ = atof(argv[14]);

    
//     my_vision.processing(frame);
// #endif
//     // close the file
//     // the camera will be deinitialized automatically in VideoCapture destructor
//     return 0;
// }

vector<Point> Vision::processing(Mat &frame) {
    // good source of image processing 
    // https://docs.opencv.org/3.4/d2/d96/tutorial_py_table_of_contents_imgproc.html
//    imwrite("test.jpg", frame);
    Mat frame_HSV;
    Mat image_threshold_hsv;
    Mat image_threshold_hsv_blur;
    Mat gray_image;
    Mat gray_image_blur;
    Mat image_BrightnessThreshold_black_obj;
    Mat image_BrightnessThreshold_white_obj;
    Mat gamma_corrected_darker=frame.clone();
    Mat gamma_corrected_hsv=frame.clone();
    Mat drawing = Mat::zeros(frame.size(), CV_8UC3);
    // --------------- Gray the image and smooth it --------------
    // convert original image to gray image and apply smoothing
    
    cvtColor(frame, gray_image, COLOR_BGR2GRAY);
    imwrite("test_gray.jpg",gray_image);
    // kernel size is 9 by 9
    blur(gray_image, gray_image_blur, Size(blur_kernel_size_, blur_kernel_size_));
    imwrite("test_gray_smooth.jpg",gray_image_blur);
#if SHOW_GRAY_IMAGE
    imshow("gray", gray_image);
    waitKey(5);
#endif
    // --------------- Gamma correction ---------------
    // it will convert the image to darker 
    gammaCorrection(gray_image_blur, gamma_corrected_darker,gamma_val_darker_);
    imwrite("test_gamma_correction_darker.jpg", gamma_corrected_darker);

    gammaCorrection(frame, gamma_corrected_hsv,gamma_val_hsv_);
    imwrite("test_gamma_correction_whiter.jpg", gamma_corrected_hsv);
    // --------------- Try image segmentation using HSV color space --------------- 
    // cout<<high_H_<<endl;
    cvtColor(gamma_corrected_hsv, frame_HSV, COLOR_BGR2HSV);
    inRange(frame_HSV, Scalar(low_H_, low_S_, low_V_), Scalar(high_H_, high_S_, high_V_), image_threshold_hsv);
    imwrite("test_hsv_img.jpg",frame_HSV);
    // blur(image_threshold_hsv, image_threshold_hsv_blur, Size(15, 15));
    imwrite("test_image_threshold_hsv.jpg",image_threshold_hsv);
    // imwrite("test_image_threshold_hsv_blur.jpg",image_threshold_hsv_blur);



    // image thresholding, there are effective 5 type of thresholding, 
    // we use the binary thresholding
    // may need to be inverted? so, the light background be come dark, 
    // the black phone become the light one
    // https://docs.opencv.org/3.4/db/d8e/tutorial_threshold.html
    // 50 is the threshold value, 255 is the max value(white)
    // if the pixel > thres, set to 0, otherwise set to 255
    // smaller threshold means the pixel need to be dark enough to be set to light
    // --------- thresholding the image ----------
    threshold(gamma_corrected_darker, image_BrightnessThreshold_black_obj, black_value_pick_up_, 255, THRESH_BINARY_INV);
    imwrite("test_threshold_black_obj.jpg", image_BrightnessThreshold_black_obj);
    vector<RotatedRect> BoundingBox = this->findBoundingBox(image_BrightnessThreshold_black_obj, drawing);
#if SHOW_THRESHOLD_IMAGE
    imshow("thresholding", image_BrightnessThreshold_black_obj);
    waitKey(5);
#endif
    // bigger threshold means the pixel need to be bright enough to be set to light
    threshold(image_threshold_hsv, image_BrightnessThreshold_white_obj, white_value_pick_up_, 255, THRESH_BINARY);
    imwrite("test_threshold_white_obj.jpg", image_BrightnessThreshold_white_obj);
    vector<RotatedRect> BoundingBox_white = this->findBoundingBox(image_BrightnessThreshold_white_obj, drawing);
    // now draw the rectangle on the mat
    //  TO DO: change it to use draw annoted function
    BoundingBox.insert(BoundingBox.end(), BoundingBox_white.begin(), BoundingBox_white.end());
    // filter the bounding box
    // draw the bounding box
    for (auto &rect: BoundingBox) {
        // color are specified in (B,G,R); 
        // draw bounding box on contour
        this->drawRotatedRect(drawing, rect, Scalar(0, 255, 255));
        // draw bounding box on original image
        this->drawRotatedRect(frame, rect, Scalar(0, 255, 255));

        // cout << "(" << rect.center.x << ", " << rect.center.y << ")    "
        //     << rect.size.width << " x " << rect.size.height << "    "
        //     << rect.angle << "°"<<endl;
    }
    imwrite( "test_contours_filter.jpg", drawing );
    imwrite("test_annoted.jpg", frame);

#if SHOW_ANNOTED_IMAGE
    imshow("annoted", frame);
    waitKey(5);
#endif
    // locate possible bounding box for the phone/airpods
    // the bounding box has been filtered
    vector<Point> locations;
    for (auto &rect: BoundingBox) {
        locations.emplace_back(Point(rect.center.x, rect.center.y));
    }

    return locations;
}


// (0,0)----x----
// |             |
// y             y
// |             |
// -------x ------
// (438.5, 2784.5)    877 x 477    -0°
// (2566.13, 2015.94)    998.062 x 509.292    -60.5153°
// (1821.13, 156.888)    42.7692 x 25.2308    -67.3801°
// (3723.5, 342)    615 x 684    -0°
// (1668.23, 1205.55)    501.65 x 535.506    -61.2602°
// (1116.1, 255.104)    348.786 x 556.306    -17.6676°
// (577.053, 79.5175)    244.472 x 592.656    -53.4007°
void Vision::gammaCorrection(const Mat &img, Mat& gamma_corrected,const double gamma_)
{
    CV_Assert(gamma_ >= 0);
    // Mat img_gamma_corrected = Mat(img.rows, img.cols, img.type());
    //! [changing-contrast-brightness-gamma-correction]
    Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for( int i = 0; i < 256; ++i){
        p[i] = saturate_cast<uchar>(pow(i / 255.0, gamma_) * 255.0);
    }
    LUT(img, lookUpTable, gamma_corrected);
    //! [changing-contrast-brightness-gamma-correction]
    // hconcat(img, res, img_gamma_corrected);
    
}

vector<RotatedRect> Vision::findBoundingBox(const Mat &image_BrightnessThreshold, Mat &drawing) {
    // find counter (it find contour of white object from black background on binary image, )
    // In OpenCV, finding contours is like finding white object from black background. 
    // So remember, object to be found should be white and background should be black.
    // Mat countour_out;
    // ------- Finding contour of image --------
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Mat coutoursOuts = image_BrightnessThreshold.clone();

    findContours(coutoursOuts, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    cout<<"before filtering, the number of contour is: "<<contours.size()<<endl;
    // cout<<"output image size "<<coutoursOuts.size()<<endl;

    for (size_t i = 0; i < contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        drawContours(drawing, contours, (int) i, color, 2, LINE_8, hierarchy, 0);
    }
#if SHOW_CONTOUR_IMAGE
    imshow( "Contours", drawing );
    waitKey(5);
#endif
    // imwrite( "test_contours.jpg", drawing );

    // ----------- find the bounding box based on contour ----------
    // Post-processing the contour, try to find the target one
    vector<RotatedRect> BoundingBox;
    for (const auto &contour : contours) {
        
        // filtering contour size and contour area
        // cout<<"checking value of max contour area"<<max_contour_area<<endl;
        // cout<<"checking value of min contour area"<<min_contour_area<<endl;
        if (contourArea(contour) > max_contour_area_ || contourArea(contour) < min_contour_area_) {
            continue;
        }
        cout << "!contour area is " << contourArea(contour) << endl;
        RotatedRect rect;
        //  rect has ( center (x,y), (width, height), angle of rotation )
        // it generate a rectangle bounding box with minimum area
        rect = minAreaRect(contour);
//        Size rect_dim=rect.size;
        int rect_size = rect.size.width * rect.size.height;
        if (rect_size < min_contour_area_ || rect_size > max_contour_area_) {
            continue;
        }


        // remove the bounding box that include whole picture
        if ((rect.size.width == (image_BrightnessThreshold.cols - 1) &&
             rect.size.height == (image_BrightnessThreshold.rows - 1))
            || (rect.size.height == (image_BrightnessThreshold.cols - 1) &&
                rect.size.width == (image_BrightnessThreshold.rows - 1))) {
            continue;
        }
        // filtering aspect ratio
        double longEdgeLength = max(rect.size.width, rect.size.height);
        double shortEdgeLength = min(rect.size.width, rect.size.height);
        double aspectRatio = longEdgeLength / shortEdgeLength;

        if (aspectRatio > aspectRatio_max_ || aspectRatio < aspectRatio_min_) {
            continue;
        }

        // filtering extend
        double extend =contourArea(contour)/rect_size;
        if (extend < extend_threshold_){
            continue;
        }
        // accept the ratio
        BoundingBox.emplace_back(rect);
        // noteContours.annotate(rect);
    }
    cout<<"number of rect "<<BoundingBox.size()<<endl;
    return BoundingBox;
}


// draw the rect on the image
void Vision::drawRotatedRect(Mat &img, const RotatedRect &rect, const Scalar &boarderColor) {
    cv::Point2f vertices[4];
    rect.points(vertices);
    for (int i = 0; i < 4; i++) {
        cv::line(img, vertices[i], vertices[(i + 1) % 4], boarderColor, 2);
    }
}

// another way of drawing contours
// Scalar colors[3];
// colors[0] = Scalar(255, 0, 0);
// colors[1] = Scalar(0, 255, 0);
// colors[2] = Scalar(0, 0, 255);
// for (size_t idx = 0; idx < contours.size(); idx++) {
//     drawContours(drawing, contours, idx, colors[idx % 3]);
// }
// https://www.jetsonhacks.com/nvidia-jetson-nano-j41-header-pinout/
// https://github.com/jwatte/jetson-gpio-example
// https://github.com/NVIDIA/jetson-gpio
// https://forums.developer.nvidia.com/t/jetson-nano-uart-c-c-example/78184
// https://forums.developer.nvidia.com/t/serial-console-setup-parameters/73073/3
// https://forums.developer.nvidia.com/t/how-to-use-uart-on-tx2/55392
// https://forums.developer.nvidia.com/t/jetson-nano-uart-c-c-example/78184/2
// https://maker.pro/nvidia-jetson/tutorial/how-to-use-gpio-pins-on-jetson-nano-developer-kit
// https://forums.developer.nvidia.com/t/jetson-nano-gpio-example-problem/75547
// https://forums.developer.nvidia.com/t/jetson-nano-fast-gpio-c-example-with-direct-register-access/79692/7
// https://forums.developer.nvidia.com/t/using-gpio-on-nvidia-jetson-tx2/58607
// https://forums.developer.nvidia.com/t/uart-communication-with-arduino-nano/83810