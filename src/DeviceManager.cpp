#include "DeviceManager.h"
#include "Common.h"

#define TABLE_WIDTH 500
#define TABLE_HEIGHT 500
#define ERROR_MARGIN 20
#define MAX_THRESHOLD 10

void DeviceManager::updateLocationMapping(const vector<cv::Point> &locations) {
    vector<cv::Point> inserted;
    vector<cv::Point> deleted;
    
    // Iterate through all new locations in the new frame
    // First check is there any new point inserted
    for (auto &point:locations) {
        
        // Check whether this point is within the margin of error
        
        bool flag = false;
        for (auto &x:locationMap) {
            
            // If there is one point in new frame can be mapped into  the old location in the map
            flag = flag || this->withinMargin(point, x.first);
            
            // If there is a point within error of margin
            if (true == flag) {
                // Update field member

                // If it reach the max count, do not update
                if (MAX_THRESHOLD == x.second) {
                    // locationMap[x.first] = MAX_THRESHOLD
                } else {
                    // If it is below the max count, add one
                    x.second += 1;
                    if (MAX_THRESHOLD == x.second) {
                        // If reach the max, put into inserted queue
                        inserted.emplace_back(x.first);
                    }
                }
                break;
            }
        }

        // If this is a totally new point, insert this point and the member count
        if (false == flag) {
            // Set the member count as 1
            locationMap.insert(std::pair<cv::Point, int>(point, 1));
        }
    }
    // Then find the deleted point

    for (auto &x:locationMap) {
        bool flag = false;
        for (auto &point:locations) {
            // Check whether location map's position is in the new point list
            flag = flag || this->withinMargin(x.first, point);
        }
        // If we don't find the map value in the new location vector
        if (false == flag) {
            x.second -= 1;
            if (0 == x.second) {
                // If reach the max, put into inserted queue
                // deleted.emplace_back(*(x.first));
                // Remove this from the map
                // locationMap.erase(*(x.first));
            }
        }
    }

}

bool DeviceManager::withinMargin(const cv::Point &curPoint, const cv::Point &refPoint) {
    // --------------x
    // | cur        |
    // |     ref    |
    // |            |
    // --------------
    // y

    if (curPoint.x > refPoint.x - ERROR_MARGIN &&
        curPoint.x < refPoint.x + ERROR_MARGIN &&
        curPoint.y > refPoint.y - ERROR_MARGIN &&
        curPoint.y < refPoint.y - ERROR_MARGIN) {
        return true;
    } else {
        return false;
    }

}

void DeviceManager::sendingLocation(const vector <cv::Point> &locations) {
    // open the file
    // /dev/ttyTHS1 is used in real case
    //  may use /dev/null as the development process, can write to it
    // write some content 
}


// We have a list of suitable candidate bounding box, we want to
// Identify the suitable one and generate the location of the bounding box
vector<cv::Point> DeviceManager::getRealLocation(const vector <cv::RotatedRect> &boundingBox, int imageWidth, int imageHeight) {
    // Here we assume that the width of image exactly include the width of table
    // The height of the image exactly include the height of table
    // We assume the upper left corner of the image is the (0,0)
    
    vector<cv::Point> realLocations;
    for (const auto &rect: boundingBox) {
        int real_x = TABLE_WIDTH * (rect.center.x / imageWidth);
        int real_y = TABLE_HEIGHT * (rect.center.y / imageHeight);
        realLocations.emplace_back(cv::Point(real_x, real_y));
    }
    return realLocations;
}