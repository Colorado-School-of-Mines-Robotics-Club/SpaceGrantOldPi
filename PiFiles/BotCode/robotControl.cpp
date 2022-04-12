//
// Created by colin on 2/17/2022.
//

#include "robotControl.h"

//
bool atBeacon = false;
//Wheel diameter = 0.16
float perMeter = 2; //This is the number of revolutions per meter

//RangFinder stuff
int toClose = 100; //checks if we are seeing an object that is too close implying an obstacle
int toFar = 1000; //checks if we are seeing an object that is too far implying an obstacle
int offset = 10; //this is the offset based off of how the angle will affect the distance
// the rangefinder should be looking at

bool scanning = false; //Tells us whether we should send a scan
bool hasScanned = false; //this tells us if we have scanned the current area that our bot is facing if it is not true we should be
// waiting for the scan before we move


bool obstacleInFront = false; //Is there an Obstacle in front of us
bool moving = false; //are we moving





//Once the wheel is done moving set moving to false
void wheelCallback(int8_t){
    moving = false;
}

//easy way to power all wheels the extra ints can be used for direction
void powerWheels(float revolutions, int first, int second, int third, int fourth) {
    moving = true;
    Wheel1->move(first*revolutions, wheelCallback);
    Wheel2->move(first*revolutions, wheelCallback);
    Wheel3->move(first*revolutions, wheelCallback);
    Wheel4->move(first*revolutions, wheelCallback);
}
//for stopping
void stopWheels(){
    moving = false;
    Wheel1->stop();
    Wheel2->stop();
    Wheel3->stop();
    Wheel4->stop();
}

//turning the bot using the rotation of the wheels
void turnBot(float degrees){
    moving = true;
    Wheel1->turnWheel(90, wheelCallback);
    Wheel3->turnWheel(-90,wheelCallback);
    while(moving){} //Wait until wheel turning done
    powerWheels(degrees,1,-1,1,1);//THIS NEEDS TO CHANGE AND DO SOME MATH TO FIGURE OUT
    //HOW MANY ROTATIONS PER DEGREE
    while(moving){} //wait until bot rotation done
    Wheel1->turnWheel(-90, wheelCallback); //Turning back to normal
    Wheel3->turnWheel(90, wheelCallback); //Turning back to normal
    while(moving){}

}

//If we can't use wheel Rotation
void tankTurn(float degrees){
    powerWheels(degrees,1,-1,-1,1);
    while(moving){}
}

void setup() {
    //Do all the checking for arduino to PI communication working/setup
    Wheel1->setPressureAlertFunction(gotBumped);
    Wheel2->setPressureAlertFunction(gotBumped);
    Wheel3->setPressureAlertFunction(gotBumped);
    Wheel4->setPressureAlertFunction(gotBumped);
}

//This should be sent into the Sensor-> scan and since it needs to be void it will set the global obstacleInFront
//to whether there is an obstacle
void checkObstacle(std::vector<RangeFinderPacket> scanPoints) {
    for(auto &point : scanPoints ){
        if(point.distance > toFar || point.distance < toClose){
            obstacleInFront = true;
        }
    }
    scanning = false;
    hasScanned = true;
}

//This is going to be running and waiting for any ping from either the bump or gyro saying there
//is a problem
void gyroControl(){
    /*if problem
     * setObstacle in front to true
     *
     *
     */
}

//This is called anytime we get a bump and its going to go backwards a fourth of a meter
void gotBumped(){
    stopWheels();
    obstacleInFront = true;
    powerWheels(perMeter/4,-1,-1,-1,-1);
    while(moving);
}

//write obstacle avoidence stuff here when entered there is an obstacle in front
void avoidObstacle(){

}



//Navigation algorithm
void navigate() {
    if(!scanning){
        sensor->scan(checkObstacle);
        scanning = true;
    }
    //we only want to start doing stuff once we have scanned
    if(hasScanned) {
        if (obstacleInFront) {
            return;
        }
        else{
            //we want to move half a meter
            powerWheels(perMeter/2, 1,1,1,1);
        }
    }


}

int main(){
    setup();
    std::thread sensorThread(gyroControl);// set up thread for bumpers and gyros we want this always going
    while(!atBeacon){
        if(!moving){ //if we aren't currently moving run our navigation again
            navigate();
            if(obstacleInFront) {
                avoidObstacle();
            }
        }
    }

}