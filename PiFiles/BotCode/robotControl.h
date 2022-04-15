//
// Created by colin on 2/17/2022.
//

#ifndef SPACEGRANTOLDPI_ROBOTCONTROL_H
#define SPACEGRANTOLDPI_ROBOTCONTROL_H

#include <vector>
#include "SensorLib.h"
//#include "../RangeFinding.h"

void setup();
void powerWheels(float revolutions, int first, int second, int third, int fourth);
void testObstacle(std::vector<RangeFinderPacket> obstacles);
int main();
void navigate();
void gotBumped();
void avoidObstacle();
void doScan();


#endif //SPACEGRANTOLDPI_ROBOTCONTROL_H
