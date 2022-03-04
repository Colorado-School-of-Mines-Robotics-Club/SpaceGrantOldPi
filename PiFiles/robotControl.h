//
// Created by colin on 2/17/2022.
//

#ifndef SPACEGRANTOLDPI_ROBOTCONTROL_H
#define SPACEGRANTOLDPI_ROBOTCONTROL_H

#include <vector>
#include "SensorLib.h"
#include "RangeFinding.h"

class robotControl {
public:
    void setup();
    void sensorCheck();
    void maintain();

    void problemLaser(std::vector<int> clump);
    void askBump();
    void problemBumper(int bumper);
    void askGyro();
    void problemGyro(std::vector<int> problem);
};
void askLaser(std::vector<RangeFinderPacket>& packets);

#endif //SPACEGRANTOLDPI_ROBOTCONTROL_H
