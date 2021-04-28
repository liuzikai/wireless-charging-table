#include "Vision.h"

#include <opencv2/opencv.hpp>

#define WRITE_IMAGES 0

// solution1: adjust the parameter ( by passing the parameter to the program )
// solution2: change to DNN
// solution3: subtract two images and find the difference

// problem1: bounding box for small object & big object
// problem2: aspect ratio change
// problem3: rect size for small object & big object
// problem4: white and black thresholding value ---> sharp change of the boundary of the object

// other image pre-processing technique beside the black & white thresholding

using namespace cv;
// height and width in millimeter

// flag for show image 
#define SHOW_ORIG_IMAGE 0
#define SHOW_GRAY_IMAGE 0
#define SHOW_THRESHOLD_IMAGE 0
#define SHOW_CONTOUR_IMAGE 0
#define SHOW_ANNOTED_IMAGE 1
#define CAMERA 0

// Communication with control TODO: volatile?


// for random color
RNG rng(12345);

Vision::Vision() {

    th = new std::thread(&Vision::runVisionThread, this);
}

void Vision::runVisionThread() {

    // Open the default camera using default API
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    cap.open(deviceID, apiID);

    // Set the resolution
    cap.set(CAP_PROP_FRAME_WIDTH, CAMERA_FRAME_WIDTH);
    cap.set(CAP_PROP_FRAME_HEIGHT, CAMERA_FRAME_HEIGHT);
    std::cout << "Camera: " << cap.get(CAP_PROP_FRAME_WIDTH) << "x" << cap.get(CAP_PROP_FRAME_HEIGHT)
              << " @ " << cap.get(cv::CAP_PROP_FPS) << " fps" << std::endl;

    // Check if we succeeded
    if (!cap.isOpened()) {
        std::cerr << "Camera: ERROR! Unable to open camera!" << std::endl;
        return;
    }

    while (true) {

        Mat frame;
        Mat frameCalibrated;

        // Wait for a new frame from camera and store it
        cap.read(frame);  // or: cap >> frame;

        // Check if we succeeded
        if (frame.empty()) {
            std::cerr << "Camera: ERROR! Blank frame grabbed" << std::endl;
            continue;
        }

        // Calibrate
        imageCalibrate(frame, frameCalibrated);
        Mat frameCalibratedCropped = frameCalibrated(
                {CROP_MARGIN_HEIGHT, CAMERA_FRAME_HEIGHT - CROP_MARGIN_HEIGHT},
                {CROP_MARGIN_WIDTH, CAMERA_FRAME_WIDTH - CROP_MARGIN_WIDTH});
//        char key = waitKey(1);
//        if (key == 'q' || key == 'Q')
//        {
//            string imgname;
//            imgname =  "ruler.jpg";
//            imwrite(imgname, frameCalibratedCropped);
//        }
#if WRITE_IMAGES
        imwrite("test.jpg", frame);
        imwrite("test_calib.jpg", frameCalibrated);
#endif

        // Run detection
        auto boxes = process(frameCalibratedCropped);

        if (!acceptImage) continue;

        // Update devices
        updateDevices(boxes);
    }
}

// calibrate the image 
// https://docs.opencv.org/master/d6/d55/tutorial_table_of_content_calib3d.html
// https://docs.opencv.org/master/d4/d94/tutorial_camera_calibration.html

void Vision::imageCalibrate(const Mat &frame, Mat &frameCalibrated) {
//    for parameter setting follow this link
//    https://blog.csdn.net/Loser__Wang/article/details/51811347
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = 1308.0;
    cameraMatrix.at<double>(0, 1) = -5.8;
    cameraMatrix.at<double>(0, 2) = 649.6;
    cameraMatrix.at<double>(1, 1) = 1314.6;
    cameraMatrix.at<double>(1, 2) = 370.3;

    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
//    first two terms are radian distortion
    distCoeffs.at<double>(0, 0) = 0.1795;
    distCoeffs.at<double>(1, 0) = -0.8530;
//    next two terms are tangantial distoration
    distCoeffs.at<double>(2, 0) = -0.0018;
    distCoeffs.at<double>(3, 0) = 1.7887e-05;
//  keep it as 0
    distCoeffs.at<double>(4, 0) = 0;

    Mat view, map1, map2;
    Size imageSize;
    imageSize = frame.size();
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                            cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                            imageSize, CV_16SC2, map1, map2);
    remap(frame, frameCalibrated, map1, map2, INTER_LINEAR);
}

vector<RotatedRect> Vision::process(Mat &frame) {
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
    Mat gamma_corrected_darker = frame.clone();
    Mat gamma_corrected_hsv = frame.clone();
    Mat image_threshold_hsv_inv;
    Mat drawing_black_obj = Mat::zeros(frame.size(), CV_8UC3);
    Mat drawing_white_obj = Mat::zeros(frame.size(), CV_8UC3);
    // --------------- Gray the image and smooth it --------------
    // convert original image to gray image and apply smoothing

    cvtColor(frame, gray_image, COLOR_BGR2GRAY);
//    imwrite("test_gray.jpg", gray_image);
    // kernel size is 9 by 9
    blur(gray_image, gray_image_blur, Size(blur_kernel_size_, blur_kernel_size_));
//    imwrite("test_gray_smooth.jpg", gray_image_blur);
#if SHOW_GRAY_IMAGE
    imshow("gray", gray_image);
    waitKey(5);
#endif
    // --------------- Gamma correction ---------------
    // it will convert the image to darker 
    gammaCorrect(gray_image_blur, gamma_corrected_darker, gamma_val_darker_);
//    imwrite("test_gamma_correction_darker.jpg", gamma_corrected_darker);

    gammaCorrect(frame, gamma_corrected_hsv, gamma_val_hsv_);
//    imwrite("test_gamma_correction_whiter.jpg", gamma_corrected_hsv);
    // --------------- Try image segmentation using HSV color space --------------- 
    // cout<<high_H_<<endl;
    cvtColor(gamma_corrected_hsv, frame_HSV, COLOR_BGR2HSV);
    inRange(frame_HSV, Scalar(low_H_, low_S_, low_V_), Scalar(high_H_, high_S_, high_V_), image_threshold_hsv);
//    imwrite("test_hsv_img.jpg", frame_HSV);
    // blur(image_threshold_hsv, image_threshold_hsv_blur, Size(15, 15));
//    imwrite("test_image_threshold_hsv.jpg", image_threshold_hsv);
    threshold(image_threshold_hsv, image_threshold_hsv_inv, black_value_pick_up_, 255, THRESH_BINARY_INV);
//    imwrite("test_image_threshold_hsv_inv.jpg", image_threshold_hsv_inv);



    // image thresholding, there are effective 5 type of thresholding, 
    // we use the binary thresholding
    // may need to be inverted? so, the light background be come dark, 
    // the black phone become the light one
    // https://docs.opencv.org/3.4/db/d8e/tutorial_threshold.html
    // 50 is the threshold value, 255 is the max value(white)
    // if the pixel > thres, set to 0, otherwise set to 255
    // smaller threshold means the pixel need to be dark enough to be set to light
    // --------- thresholding the image ----------
    threshold(gamma_corrected_darker, image_BrightnessThreshold_black_obj, black_value_pick_up_, 255,
              THRESH_BINARY_INV);
//    imwrite("test_threshold_black_obj.jpg", image_BrightnessThreshold_black_obj);

    vector<vector<Point>> contours_black = findAndDrawContours(image_BrightnessThreshold_black_obj, drawing_black_obj);
    // draw_contours(contours_black, drawing_black_obj);
//    imwrite("test_contours_black_obj.jpg", drawing_black_obj);
    vector<RotatedRect> BoundingBox = findBoundingBoxes(image_BrightnessThreshold_black_obj, contours_black);
#if SHOW_THRESHOLD_IMAGE
    imshow("thresholding", image_BrightnessThreshold_black_obj);
    waitKey(5);
#endif
    // bigger threshold means the pixel need to be bright enough to be set to light
    threshold(image_threshold_hsv_inv, image_BrightnessThreshold_white_obj, white_value_pick_up_, 255, THRESH_BINARY);
//    imwrite("test_threshold_white_obj.jpg", image_BrightnessThreshold_white_obj);
    vector<vector<Point>> contours_white = findAndDrawContours(image_BrightnessThreshold_white_obj, drawing_white_obj);
    // draw_contours(contours_white, drawing_white_obj);
//    imwrite("test_contours_white_obj.jpg", drawing_white_obj);
    vector<RotatedRect> BoundingBox_white = findBoundingBoxes(image_BrightnessThreshold_white_obj, contours_white);
    // now draw the rectangle on the mat

    drawBoundingBoxes(BoundingBox, drawing_black_obj, frame);
//    imwrite("test_bouding_box_black_obj.jpg", frame);
    drawBoundingBoxes(BoundingBox, drawing_white_obj, frame);
//    imwrite("test_bouding_box_white+black_obj.jpg", frame);
    //  TO DO: change it to use draw annoted function
    BoundingBox.insert(BoundingBox.end(), BoundingBox_white.begin(), BoundingBox_white.end());
    // filter the bounding box


    // draw the bounding box
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

    return BoundingBox;
}

void Vision::drawBoundingBoxes(vector<RotatedRect> &boundingBoxes, Mat &drawing, Mat &frame) {
    for (auto &rect: boundingBoxes) {
        // color are specified in (B,G,R); 
        // draw bounding box on contour
        this->drawRotatedRect(drawing, rect, Scalar(0, 255, 255));
        // draw bounding box on original image
        this->drawRotatedRect(frame, rect, Scalar(0, 255, 255));

        // cout << "(" << rect.center.x << ", " << rect.center.y << ")    "
        //     << rect.size.width << " x " << rect.size.height << "    "
        //     << rect.angle << "°"<<endl;
        // imwrite( "test_contours_filter_obj.jpg", drawing );
        // imwrite("test_annoted_obj.jpg", frame);
    }
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
void Vision::gammaCorrect(const Mat &img, Mat &gammaCorrected, double gamma) {
    CV_Assert(gamma >= 0);
    // Mat img_gamma_corrected = Mat(img.rows, img.cols, img.type());
    //! [changing-contrast-brightness-gamma-correction]
    Mat lookUpTable(1, 256, CV_8U);
    uchar *p = lookUpTable.ptr();
    for (int i = 0; i < 256; ++i) {
        p[i] = saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
    }
    LUT(img, lookUpTable, gammaCorrected);
    //! [changing-contrast-brightness-gamma-correction]
    // hconcat(img, res, img_gamma_corrected);

}

vector<vector<Point>> Vision::findAndDrawContours(const cv::Mat &brightnessThreshold, Mat &drawing) {
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Mat coutoursOuts = brightnessThreshold.clone();

    findContours(coutoursOuts, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//    cout << "before filtering, the number of contour is: " << contours.size() << endl;

    for (size_t i = 0; i < contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        drawContours(drawing, contours, (int) i, color, 2, LINE_8, hierarchy, 0);
    }
#if SHOW_CONTOUR_IMAGE
    imshow( "Contours", drawing );
    waitKey(5);
#endif
    return contours;
}

vector<RotatedRect> Vision::findBoundingBoxes(const Mat &brightnessThreshold, vector<vector<Point>> &contours) {
    // find counter (it find contour of white object from black background on binary image, )
    // In OpenCV, finding contours is like finding white object from black background. 
    // So remember, object to be found should be white and background should be black.
    // Mat countour_out;
    // ------- Finding contour of image --------

    // cout<<"output image size "<<coutoursOuts.size()<<endl;

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
//        cout << "!contour area is " << contourArea(contour) << endl;
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
        if ((rect.size.width == (brightnessThreshold.cols - 1) &&
             rect.size.height == (brightnessThreshold.rows - 1))
            || (rect.size.height == (brightnessThreshold.cols - 1) &&
                rect.size.width == (brightnessThreshold.rows - 1))) {
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
        double extend = contourArea(contour) / rect_size;
        if (extend < extend_threshold_) {
            continue;
        }
        // accept the ratio
        BoundingBox.emplace_back(rect);
        // noteContours.annotate(rect);
    }
//    cout << "number of rect " << BoundingBox.size() << endl;
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

void Vision::updateDevices(const vector<cv::RotatedRect> &boxes) {

    vector<bool> boxMatched(boxes.size(), false);  // bool table for boxes

    // Update existing devices
    for (auto &it : existingDevices) {
        Device &device = it.second;

        // Match existing devices in last update with currently detected boxes
        int i;
        for (i = 0; i < boxes.size(); i++) {
            if (rectMatched(boxes[i], device.rect)) {
                break;
            }
        }

        if (i < boxes.size()) {  // matched
            boxMatched[i] = true;

            // Update existing device
            device.rect = combineRect(boxes[i], device.rect);
            if (device.counter < COUNTER_THRESHOLD) device.counter++;

        } else {  // no match

            if (device.counter > 0) device.counter--;
            // The item will be needed and erased at fetchDeviceDiff

        }
    }

    // Insert new device
    for (int i = 0; i < boxes.size(); i++) {
        if (!boxMatched[i]) {
            existingDevices.emplace(nextUID++, Device{boxes[i], 1, false});
        }
    }
}

void Vision::fetchDeviceDiff(vector<cv::RotatedRect> &newDevices, vector<cv::RotatedRect> &deletedDevices) {

    for (auto it = existingDevices.begin(); it != existingDevices.cend() /* not hoisted */; /* no increment */) {
        Device &device = it->second;
        bool shouldDelete = false;

        const RotatedRect &realRect = getRealRect(device.rect);
        if (!device.reported && device.counter == COUNTER_THRESHOLD) {
            // A device newly reached COUNTER_THRESHOLD
            std::cout << "Vision: report inserted " << "(" << realRect.center.x << "," << realRect.center.y << ") "
                      << realRect.size.width << "x" << realRect.size.height << "%" << realRect.angle
                      << std::endl;
            newDevices.emplace_back(realRect);
            device.reported = true;
        } else if (device.reported && device.counter == 0) {
            // A device reported but no longer exist
            std::cout << "Vision: report deleted " << "(" << realRect.center.x << "," << realRect.center.y << ") "
                      << realRect.size.width << "x" << realRect.size.height << "%" << realRect.angle
                      << std::endl;
            deletedDevices.emplace_back(realRect);
            shouldDelete = true;
        }

        if (shouldDelete) {
            it = existingDevices.erase(it);
        } else {
            ++it;
        }
    }
}

// We have a list of suitable candidate bounding box, we want to
// Identify the suitable one and generate the location of the bounding box
cv::RotatedRect Vision::getRealRect(cv::RotatedRect &old_rect) {
    // Here we assume that the width of image exactly include the width of table
    // The height of the image exactly include the height of table
    // We assume the upper left corner of the image is the (0,0)

    cv::RotatedRect new_rect = old_rect;
    new_rect.center.x = (old_rect.center.x / 2.15f);
    new_rect.center.y = (old_rect.center.y / 2.15f);
    new_rect.size.width = (old_rect.size.width / 2.15f);
    new_rect.size.height = (old_rect.size.height / 2.15f);
    return new_rect;
}


bool Vision::pointClose(const cv::Point &cur, const cv::Point &ref) {
    return ((cur.x > ref.x - COORDINATE_OFFSET_THRESHOLD) &&
            (cur.x < ref.x + COORDINATE_OFFSET_THRESHOLD) &&
            (cur.y > ref.y - COORDINATE_OFFSET_THRESHOLD) &&
            (cur.y < ref.y + COORDINATE_OFFSET_THRESHOLD));
}

bool Vision::rectMatched(const cv::RotatedRect &cur, const cv::RotatedRect &ref) {

    // Filter by central point
    if (!pointClose(cur.center, ref.center)) return false;

    // Filter by area size
    float areaRatio = cur.size.area() / ref.size.area();
    if (areaRatio < AREA_RATIO_THRESHOLD || areaRatio > 1 / AREA_RATIO_THRESHOLD) return false;

    // Filter by intersection area
    vector<cv::Point2f> points;
    if (cv::rotatedRectangleIntersection(cur, ref, points) == cv::INTERSECT_NONE) return false;
    auto intersectionArea = cv::contourArea(points);
    if (intersectionArea / cur.size.area() < INTERSECTION_AREA_THRESHOLD) return false;

    return true;
}

cv::RotatedRect Vision::combineRect(const cv::RotatedRect &cur, const cv::RotatedRect &ref) {
    cv::Point2f c(cur.center.x * ROTATED_RECT_UPDATE_RATE + ref.center.x * (1 - ROTATED_RECT_UPDATE_RATE),
                  cur.center.y * ROTATED_RECT_UPDATE_RATE + ref.center.y * (1 - ROTATED_RECT_UPDATE_RATE));
    cv::Size s(cur.size.width * ROTATED_RECT_UPDATE_RATE + ref.size.width * (1 - ROTATED_RECT_UPDATE_RATE),
               cur.size.height * ROTATED_RECT_UPDATE_RATE + ref.size.height * (1 - ROTATED_RECT_UPDATE_RATE));
    float a = cur.angle * ROTATED_RECT_UPDATE_RATE + ref.angle * (1 - ROTATED_RECT_UPDATE_RATE);
    return {c, s, a};
}