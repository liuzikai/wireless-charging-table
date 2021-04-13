//
// Created by liuzikai on 2/20/21.
//

#ifndef COMMON_H
#define COMMON_H

#include <string>
#include <vector>

using std::string;
using std::stringstream;
using std::vector;



template<typename T>
struct Range {
    T min;  // inclusive
    T max;  // inclusive
    bool contains(const T &value) const { return min <= value && value <= max; }
};

struct SharedParameters {
    int imageWidth = 500;
    int imageHeight = 500;
};



#endif //COMMON_H