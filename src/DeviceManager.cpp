#include "DeviceManager.h"
#include "Common.h"
#include <iostream>
#define TABLE_WIDTH 500
#define TABLE_HEIGHT 500
#define ERROR_MARGIN 20
#define MAX_THRESHOLD 10
using namespace std;
void DeviceManager::updateLocationMapping(const vector<cv::Point> &locations, vector<cv::Point>& inserted, vector<cv::Point>& deleted) {

    
    // Iterate through all new locations in the new frame
    // First check is there any new point inserted
    for (auto &point:locations) {
        
        // Check whether this point is within the margin of error
        
        bool flag = false;
        for (auto &x:locationMap) {
            
            // If there is one point in new frame can be mapped into  the old location in the map
            flag = flag || this->withinMargin(point, x.first);
            // cout<<" point value "<<point<<" location map value "<<x.first<<" flag value " <<flag<<endl;
            // If there is a point within error of margin
            if (true == flag) {
                // Update field member

                // If it reach the max count, do not update
                if (MAX_THRESHOLD == x.second) {
                    // locationMap[x.first] = MAX_THRESHOLD
                } else {
                    // If it is below the max count, add one
                    x.second += 1;
                    // cout<<"reach increse member count !!!"<<endl;
                    if (MAX_THRESHOLD == x.second) {
                        // If reach the max, put into inserted queue
                        inserted.emplace_back(x.first);
                        presentedDevice.emplace_back(x.first);
                    }
                }
                break;
            }
        }

        // If this is a totally new point, insert this point and the member count
        if (false == flag) {
            // Set the member count as 1
            locationMap.emplace_back(std::pair<cv::Point, int>{point, 1});
        }
    }
    // cout<< " after the insertion update "<<locationMap.size()<<endl;
    // for (auto &x:locationMap) {
        // cout<<"pos "<<x.first<<" member count "<<x.second<<endl;
    // }
    // need to check member count
    // Then find the deleted point
    vector <std::pair<cv::Point, int>> locationMap_new;
    // https://www.techiedelight.com/remove-elements-vector-inside-loop-cpp/
    for (auto &x:locationMap) {
        bool flag = false;   
        
        for (auto &point:locations) {
            // Check whether location map's position is in the new point list
            flag = flag || this->withinMargin(x.first, point);
        }
        // cout<<" pos in map (check remove) "<<x.first<<" point value "<<point<<" flag value " <<flag<<endl;
        // If we don't find the map value in the new location vector
        if (false == flag) {
            // means this location is no longer found in the 
            if (0 == x.second) {
                // If reach the max, put into deleted queue
                // iterate through presentedDevice to see whether we really 
                // need to remove it or this device to be removed is just a sudden change
                bool deleteFlag=false;
                for (auto& point:presentedDevice){
                    deleteFlag = deleteFlag || this->withinMargin(x.first, point);
                }
                if (true==deleteFlag){
                    deleted.emplace_back(x.first);
                    // Remove this from the map
                    // cout<<"going to delete this point"<<endl;
                }
            }else{
                // cout<<"going to decrease the member count"<<endl;
                x.second -= 1;
                locationMap_new.emplace_back(x);
            }  
        }else{
            // cout<<" update location map "<<endl;
            locationMap_new.emplace_back(x);
        }
    }
    
    locationMap=locationMap_new;
    // cout<<" after the deletion update "<<locationMap.size()<<endl;
    // for (auto &x:locationMap) {
    //     cout<<"pos "<<x.first<<" member count "<<x.second<<endl;
    // }
}

bool DeviceManager::withinMargin(const cv::Point &curPoint, const cv::Point &refPoint) {
    // --------------x
    // | cur        |
    // |     ref    |
    // |            |
    // --------------
    // y
    // cout<<"first arg "<<curPoint.x<<" "<<curPoint.y<<endl;
    // cout<<"second arg "<<refPoint.x<<"  "<<refPoint.y<<endl;
    // cout<<" 1 "<<(curPoint.x > refPoint.x - ERROR_MARGIN) << endl;
    // cout<<" 2 "<<(curPoint.x < refPoint.x + ERROR_MARGIN) <<endl;
    // cout<<" 3 "<<(curPoint.y > refPoint.y - ERROR_MARGIN) <<endl;
    // cout<<" 4 "<<(curPoint.y < refPoint.y - ERROR_MARGIN) <<endl;
    if ((curPoint.x > refPoint.x - ERROR_MARGIN) &&
        (curPoint.x < refPoint.x + ERROR_MARGIN) &&
        (curPoint.y > refPoint.y - ERROR_MARGIN) &&
        (curPoint.y < refPoint.y + ERROR_MARGIN) ) {
        return true;
    } else {
        return false;
    }

}

void DeviceManager::sendingLocation(const vector<cv::Point>& inserted, const vector<cv::Point>& deleted) {
    // open the file
    // /dev/ttyTHS1 is used in real case
    //  may use /dev/null as the development process, can write to it
    // write some content 
}


// We have a list of suitable candidate bounding box, we want to
// Identify the suitable one and generate the location of the bounding box
vector<cv::Point> DeviceManager::getRealLocation(const vector <cv::Point> &locations, int imageWidth, int imageHeight) {
    // Here we assume that the width of image exactly include the width of table
    // The height of the image exactly include the height of table
    // We assume the upper left corner of the image is the (0,0)
    
    vector<cv::Point> realLocations;
    for (const auto &point: locations) {
        int real_x = TABLE_WIDTH * (point.x / imageWidth);
        int real_y = TABLE_HEIGHT * (point.y / imageHeight);
        realLocations.emplace_back(cv::Point(real_x, real_y));
    }
    return realLocations;
}