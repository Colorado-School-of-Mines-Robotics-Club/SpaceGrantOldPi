//
// Created by colin on 2/17/2022.
//

#include "robotControl.h"
#include "SensorLib.h"

//
// Created by colin on 2/10/2022.
//

#include "robotControl.h"






//figures out motor commands to go towards beacon
void robotControl::maintain() {

}

//does a general sensor check
void robotControl::sensorCheck() {
    askBump();
    askGyro();
    sensor->scan(askLaser);
}

//uses I2C and if something is wrong gets a vector with values and passes to problem
void robotControl::askGyro() {
    std::vector<int> tester;
    problemGyro(tester);
}
//This will be called if there is an issue and send commands to motors to avoid
void robotControl::problemGyro(std::vector<int> problem) {

}

//Checks if any bumpers are triggered then passes the bumper if is to problem
void robotControl::askBump() {
    problemBumper(1);
}
//sends commands to motor to get away;
void robotControl::problemBumper(int bumper) {

}

//Asks Laser and gets a vector of clumps
//if clumps is not empty sends to problem laser
void askLaser(std::vector<RangeFinderPacket>& packets){
    //Put a call to your function here
}
//adjusts path to avoid obstacle shown by clumps
void robotControl::problemLaser(std::vector<int> clumps) {
    //solve laser problem
}


void robotControl::setup() {
    //send signal to arduinos for calibration
    //swivel rangefinder
    //calibrate gyro
    //check beacon heading
}

