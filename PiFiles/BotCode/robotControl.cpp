//
// Created by colin on 2/17/2022.
//

#include "robotControl.h"

//
bool atBeacon = false;

//Range finder mapping stuff might just chang this to only be obstacleInFront
int rangeFinderRadius = 10; //This is the largest distance the Rangefinder can see
int MapSize = 21; //This is the dimensions of the map and should be odd bigger dimensions more precise
std::vector<std::vector<bool>> Map(MapSize, std::vector<bool>(MapSize,0)); //This is the initial Map

bool obstacleInFront = false; //Is there an Obstacle in front of us
bool hasScanned = false; //this tells us if we have scanned the current area that our bot is facing if it is not true we should be
// waiting for the scan before we move
bool moving = false; //are we moving




//Once the wheel is done moving set moving to false
void wheelCallback(int8_t){
    moving = false;
}

//easy way to turn all wheels in the same direction
void powerWheels(float revolutions) {
    moving = true;
    Wheel1->move(revolutions, wheelCallback);
    Wheel2->move(revolutions, wheelCallback);
    Wheel3->move(revolutions, wheelCallback);
    Wheel4->move(revolutions, wheelCallback);
}


//uses I2C and if something is wrong gets a vector with values and passes to problem
void askGyro() {
    std::vector<int> tester;
    problemGyro(tester);
}
//This will be called if there is an issue and send commands to motors to avoid
void problemGyro(std::vector<int> *problem) {

}


void setup() {
    //Do all the checking for arduino to PI communication working/setup
}

//This should be sent into the Sensor-> scan and since it needs to be void it will set the global obstacleInFront
//to whether there is an obstacle
void testObstacle(std::vector<RangeFinderPacket> *obstacles) {
    //Resetting Map
    for(int i = 0; i < MapSize; i++){
        for(int j = 0; j < MapSize; j++){
            Map[i][j] = false;
        }
    }
    obstacleInFront = false;
    int botX = int(MapSize/2);
    int botY = int(MapSize/2);
    //Updating Map to reflect the scan
    for(auto & point : *obstacles){
        int yOffset = round(sin(point.angle)*MapSize/rangeFinderRadius); //Calculates distance from bot in terms
        int xOffset = round(cos(point.angle)*MapSize/rangeFinderRadius); //Of indices
        Map[botY+yOffset][botX+xOffset] = true; //Registers there is a obstacle at this location
    }
}

//This is going to be running and waiting for any ping from either the bump or gyro saying there
//is a problem
void bumpGyro(){
    /*if problem
     * setObstacle in front to true
     */
}


void navigate() {

}

int main(){
    setup();
    std::thread sensorThread(bumpGyro);// set up thread for bumpers and gyros we want this always going
    while(!atBeacon){
        if(hasScanned){ //if we have scanned the part infront of us
            std::thread navigateThread(navigate); //starts a new thread for navigation
            navigateThread.join(); //Wait until the navigate thread is done
        }
    }

}