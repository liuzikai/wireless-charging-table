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

#define GPU 0

#if GPU
#include <opencv2/gpu/gpu.hpp>  
#endif


using namespace cv;
using namespace std;
// height and width in millimeter
#define TABLE_WIDTH 800
#define TABLE_HEIGHT 500
#define FRAME_WIDTH 800
#define FRAME_HEIGHT 500


// flag for show image 
#define SHOW_ORIG_IMAGE 0
#define SHOW_GRAY_IMAGE 0
#define SHOW_THRESHOLD_IMAGE 0
#define SHOW_CONTOUR_IMAGE 0
#define SHOW_ANNOTED_IMAGE 1
#define CAMERA 1

// parameter used in code
#define BLUR_KERNEL_SIZE 9
#define MIN_CONTOUR_AREA 50
#define MIN_CONTOUR_PIXEL 50


// for random color
RNG rng(12345);

// helper function 
void processing(Mat &frame);

void drawRotatedRect(Mat &img, const RotatedRect &rect, const Scalar &boarderColor);

vector <RotatedRect> find_bounding_box(Mat &image_BrightnessThreshold, Mat &drawing);

vector <Point> get_real_location(vector <RotatedRect> &BoundingBox, int image_width, int image_height);

void sending_location(vector <Point> locations);
// calibrate the image 
// https://docs.opencv.org/master/d6/d55/tutorial_table_of_content_calib3d.html
// https://docs.opencv.org/master/d4/d94/tutorial_camera_calibration.html

// set the file descriptor as a 


int main(int argc, char **argv) {
    // open the file for data transmission

#if CAMERA
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
    cap.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    cout << "frame width is " << cap.get(CAP_PROP_FRAME_WIDTH) <<
         "frame height is " << cap.get(CAP_PROP_FRAME_HEIGHT) << endl;
    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }
    //--- GRAB AND WRITE LOOP ----- 
    cout << "Start streaming the video" << endl;

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
        imwrite("test.jpg", frame);
        // call a function to process the image, passing by reference
        processing(frame);
    }
#endif

#if !CAMERA
    // read from the cmd arg
    if( argc != 2) {
        cout <<" Usage: ./detector image_to_process" << endl;
        return -1;
    }
    Mat frame=imread(argv[1]); 
    if (frame.empty()) {
        cerr << "ERROR! blank frame grabbed\n";
    }
    // imshow("Live", frame);
    processing(frame);
#endif
    // close the file


    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}

void processing(Mat &frame) {
    // good source of image processing 
    // https://docs.opencv.org/3.4/d2/d96/tutorial_py_table_of_contents_imgproc.html
    Mat gray_image;
    Mat gray_image_blur;
    Mat image_BrightnessThreshold_black_obj;
    Mat image_BrightnessThreshold_white_obj;
    // -------- Gray the image and smooth it -------
    // convert original image to gray image and apply smoothing
    cvtColor(frame, gray_image, COLOR_BGR2GRAY);
    // kernel size is 9 by 9
    // imwrite("test_gray.jpg",gray_image);
    blur(gray_image, gray_image_blur, Size(BLUR_KERNEL_SIZE, BLUR_KERNEL_SIZE));
    // imwrite("test_gray_smooth.jpg",gray_image_blur);
#if SHOW_GRAY_IMAGE
    imshow("gray", gray_image);
    waitKey(5);
#endif

    // image thresholding, there are effective 5 type of thresholding, 
    // we use the binary thresholding
    // may need to be inverted? so, the light background be come dark, 
    // the black phone become the light one
    // https://docs.opencv.org/3.4/db/d8e/tutorial_threshold.html
    // 50 is the threshold value, 255 is the max value(white)
    // if the pixel > thres, set to 0, otherwise set to 255
    // smaller threshold means the pixel need to be dark enough to be set to light
    // --------- thresholding the image ----------
    threshold(gray_image_blur, image_BrightnessThreshold_black_obj, 50, 255, THRESH_BINARY_INV);
    imwrite("test_threshold_black_obj.jpg", image_BrightnessThreshold_black_obj);
#if SHOW_THRESHOLD_IMAGE
    imshow("thresholding", image_BrightnessThreshold_black_obj);
    waitKey(5);
#endif
    Mat drawing = Mat::zeros(image_BrightnessThreshold_black_obj.size(), CV_8UC3);
    vector <RotatedRect> BoundingBox = find_bounding_box(image_BrightnessThreshold_black_obj, drawing);


    threshold(gray_image_blur, image_BrightnessThreshold_white_obj, 210, 255, THRESH_BINARY);
    imwrite("test_threshold_white_obj.jpg", image_BrightnessThreshold_white_obj);
    vector <RotatedRect> BoundingBox_white = find_bounding_box(image_BrightnessThreshold_white_obj, drawing);
    // now draw the rectangle on the mat
    //  TO DO: change it to use draw annoted function
    BoundingBox.insert(BoundingBox.end(), BoundingBox_white.begin(), BoundingBox_white.end());
    // filter the bounding box
    // draw the bounding box
    for (auto &rect: BoundingBox) {
        // color are specified in (B,G,R); 
        // draw bounding box on contour
        drawRotatedRect(drawing, rect, Scalar(0, 255, 255));
        // draw bounding box on original image
        drawRotatedRect(frame, rect, Scalar(0, 255, 255));

        // cout << "(" << rect.center.x << ", " << rect.center.y << ")    "
        //     << rect.size.width << " x " << rect.size.height << "    "
        //     << rect.angle << "°"<<endl;
    }
    // imwrite( "test_contours_filter.jpg", drawing );
    imwrite("test_annoted.jpg", frame);

#if SHOW_ANNOTED_IMAGE
    imshow("annoted", frame);
    waitKey(5);
#endif
    // locate possible bounding box for the phone/airpods
    // the bounding box has been filtered
    vector <Point> locations = get_real_location(BoundingBox, frame.cols, frame.rows);
    for (auto &point: locations) {
        cout << "(" << point.x << "," << point.y << ")" << endl;
    }
    sending_location(locations);
}


void sending_location(vector <Point> locations) {
    // open the file
    // /dev/ttyTHS1 is used in real case
    //  may use /dev/null as the development process, can write to it



    // write some content 



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


vector <RotatedRect> find_bounding_box(Mat &image_BrightnessThreshold, Mat &drawing) {
    // find counter (it find contour of white object from black background on binary image, )
    // In OpenCV, finding contours is like finding white object from black background. 
    // So remember, object to be found should be white and background should be black.
    // Mat countour_out;
    // ------- Finding contour of image --------
    vector <vector<Point>> contours;
    vector <Vec4i> hierarchy;
    Mat coutours_out = image_BrightnessThreshold.clone();

    findContours(coutours_out, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    // cout<<"number of contour "<<contours.size()<<endl;
    // cout<<"output image size "<<coutours_out.size()<<endl;

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
    vector <RotatedRect> BoundingBox;
    for (const auto &contour : contours) {

        // filtering contour size and contour area
        if (contour.size() < MIN_CONTOUR_PIXEL || contourArea(contour) < MIN_CONTOUR_AREA) {
            continue;
        }
        RotatedRect rect;
        //  rect has ( center (x,y), (width, height), angle of rotation )
        // it generate a rectangle bounding box with minimum area
        rect = minAreaRect(contour);
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

        if (aspectRatio > 2.5) {
            continue;
        }
        // accept the ratio
        BoundingBox.emplace_back(rect);
        // noteContours.annotate(rect);
    }
    // cout<<"number of rect "<<BoundingBox.size()<<endl;
    return BoundingBox;
}


// we have a list of suitable candidate bounding box, we want to
// identify the suitable one and generate the location of the bounding box
vector <Point> get_real_location(vector <RotatedRect> &BoundingBox, int image_width, int image_height) {
    // here we assume that the width of image exactly include the width of table
    // the height of the image exactly include the height of table
    // we assume the upper left corner of the image is the (0,0)
    vector <Point> real_locations;
    for (auto &rect: BoundingBox) {
        int real_x = TABLE_WIDTH * (rect.center.x / image_width);
        int real_y = TABLE_HEIGHT * (rect.center.y / image_height);
        real_locations.emplace_back(Point(real_x, real_y));
    }
    return real_locations;
}


// draw the rect on the image
void drawRotatedRect(Mat &img, const RotatedRect &rect, const Scalar &boarderColor) {
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