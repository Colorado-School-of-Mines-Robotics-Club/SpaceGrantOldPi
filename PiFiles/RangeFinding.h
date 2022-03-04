//
// Created by Cailyn Smith on 2/17/22.
//

#ifndef SPACEGRANTOLDPI_RANGEFINDING_H
#define SPACEGRANTOLDPI_RANGEFINDING_H
#include <iostream>
#include <list>
#include <vector>
#include "SensorLib.h"
struct Clump{
    RangeFinderPacket point1;
    RangeFinderPacket point2;
};

class RangeFinding {
public:
    std::vector<Clump>* RangeFinder (std::vector<RangeFinderPacket> data);
};


#endif //SPACEGRANTOLDPI_RANGEFINDING_H
