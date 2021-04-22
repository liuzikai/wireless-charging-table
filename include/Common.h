//
// Created by liuzikai on 4/13/21.
//

#ifndef COMMON_H
#define COMMON_H

#include <string>
#include <vector>
#include <iostream>

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
    int imageWidth = 1280;
    int imageHeight = 720;
};



#endif //COMMON_H