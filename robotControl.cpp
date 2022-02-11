//
// Created by colin on 2/10/2022.
//

#include "robotControl.h"

void setup();
void sensorCheck();
void maintain();
void askLaser();
void problemLaser(vector<int> clump);
void askBump();
void problemBumper(int bumper);
void askGyro();
void problemGyro(vector<int> problem);



using namespace std;

int main(){
    bool running = true;
    setup();
    int i = 0;
    while(running){
        if(i = 100){
            running =false;
        }
        sensorCheck();
        maintain();
        i++;
    }
}

//figures out motor commands to go towards beacon
void maintain() {

}

//does a general sensor check
void sensorCheck() {
    askBump();
    askGyro();
    askLaser();
}

//uses I2C and if something is wrong gets a vector with values and passes to problem
void askGyro() {
    vector<int> tester;
    problemGyro(tester);
}
//This will be called if there is an issue and send commands to motors to avoid
void problemGyro(vector<int> problem) {

}

//Checks if any bumpers are triggered then passes the bumper if is to problem
void askBump() {
    problemBumper(1);
}
//sends commands to motor to get away;
void problemBumper(int bumper) {

}

//Asks Laser and gets a vector of clumps
//if clumps is not empty sends to problem laser
void askLaser(){
    vector<int> tester;
    problemLaser(tester);
}
//adjusts path to avoid obstacle shown by clumps
void problemLaser(vector<int> clumps) {
    //solve laser problem
}


void setup() {
    //send signal to arduinos for calibration
    //swivel rangefinder
    //calibrate gyro
    //check beacon heading
}

