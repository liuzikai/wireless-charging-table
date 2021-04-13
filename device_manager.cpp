#include "device_manager.h"
#include "Common.h"


#define TABLE_WIDTH 500
#define TABLE_HEIGHT 500
#define ERROR_MARGIN 20
#define MAX_THRESHOLD 10
using namespace std;


void Device_manager::update_location_mapping(vector <Point> locations){
    
    vector <Point> inserted;
    vector <Point> deleted;
    // iterate through all new locations in the new frame
    // first check is there any new point inserted 
    for (auto& point:locations){
        // check whether this point is within the margin of error
        bool flag=false;
        for(auto& x:location_map){
            // if there is one point in new frame can be mapped into 
            // the old location in the map
            flag=flag || this->within_margin(point, x.first);
            // if there is a point within error of margin
            if (true==flag){
                // update field member 
                // if it reach the max count, do not update
                if(MAX_THRESHOLD==x.second){
                    // location_map[x.first] = MAX_THRESHOLD
                }else{
                    // if it is below the max count, add one
                    x.second+=1;
                    if(MAX_THRESHOLD==x.second){
                        // if reach the max, put into inserted queue
                        inserted.emplace_back(x.first);
                    }
                }
                break;
            }
        }
        // if this is a totally new point
        // insert this point and the member count
        if (false==flag){
            // set the member count as 1
            location_map.insert(std::pair<Point,int>(point,1));
        }     
    }
    // then find the deleted point

    for (auto& x:location_map){
        bool flag=false;
        for(auto& point:locations){
            // check whether location map's position is in the new point list
            flag=flag || this->within_margin(x.first,point);
        }
        // if we don't find the map value in the new location vector
        if (false==flag){
            x.second-=1;
            if(0==x.second){
                // if reach the max, put into inserted queue
                // deleted.emplace_back(*(x.first));
                // remove this from the map
                // location_map.erase(*(x.first));
            }
        }  
    }

}

bool Device_manager::within_margin(const Point& cur_point, const Point& ref_point){
    // --------------x
    // | cur        |
    // |     ref    |
    // |            |
    // --------------
    // y
    if( cur_point.x > ref_point.x - ERROR_MARGIN &&
        cur_point.x < ref_point.x + ERROR_MARGIN &&
        cur_point.y > ref_point.y - ERROR_MARGIN && 
        cur_point.y < ref_point.y - ERROR_MARGIN ){
        return true;
    }else{
        return false;
    }

}

void Device_manager::sending_location(vector <Point> locations) {
    // open the file
    // /dev/ttyTHS1 is used in real case
    //  may use /dev/null as the development process, can write to it
    // write some content 
}


// we have a list of suitable candidate bounding box, we want to
// identify the suitable one and generate the location of the bounding box
vector <Point> Device_manager::get_real_location(vector <RotatedRect> &BoundingBox, int image_width, int image_height) {
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