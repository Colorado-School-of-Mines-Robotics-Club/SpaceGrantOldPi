//
// Created by colin on 2/17/2022.
//
#ifndef robotControl_cpp
#define robotControl_cpp

#include "robotControl.h"
#define dist(A, B, N) A - B > N - (A - B - 1) ? fmin(A - B, N - (A - B - 1)) : -fmin(A - B, N - (A - B - 1))

// if we get to Beacon we are done
bool atBeacon = false;
//Gyro stuff
float initialX =0;
float initialY =0;
float initialZ =0;

//RangFinder stuff
int toClose = 100; //checks if we are seeing an object that is too close implying an obstacle
int toFar = 1000; //checks if we are seeing an object that is too far implying an obstacle
int offset = 10; //this is the offset based off of how the angle will affect the distance
// the rangefinder should be looking at

bool scanning = false; //Tells us whether we should send a scan
bool hasScanned = false; //this tells us if we have scanned the current area that our bot is facing if it is not true we should be
// waiting for the scan before we move
int direction = -1; //Really bad way to do this but for now if this is -1 we default
//to avoid left first and if 1 we default to right would be good if we could replace so
//we default to go towards the beacon

//Movement stuff
bool obstacleInFront = false; //Is there an Obstacle in front of us
bool moving = false; //are we moving
bool avoiding = false; //If this is triggered we should not be navigating

//Wheel diameter = 0.16
float perMeter = 2; //This is the number of revolutions per meter


void setup() {
    // General gpio initialization
    if(gpioInitialise() < 0){
        std::cout << "GPIO failed to initialize!" << std::endl;
    }else{
        std::cout << "GPIO initialized!" << std::endl;
    }
    sensor = new Sensor();
    //Do all the checking for arduino to PI communication working/setup
//    Wheel1 = new Wheel(0x1A,1,24);
//    Wheel2 = new Wheel(0x1B,1,25);
//    Wheel3 = new Wheel(0x1C,1,8);
//    Wheel4 = new Wheel(0x1D,1,23);
     
    sensor->setPressureAlertFunction(gotBumped);

    Vector3 initial = sensor->getRotation();
    initialX = initial.x;
    initialY = initial.y;
    initialZ = initial.z;

}

//Once the wheel is done moving set moving to false
void wheelCallback(int8_t){
    moving = false;
}

//easy way to power all wheels the extra ints can be used for direction
void powerWheels(float revolutions, int first, int second, int third, int fourth) {
    moving = true;
    sensor->moveWheel(first * revolutions, wheelCallback,0);
    sensor->moveWheel(second * revolutions, wheelCallback,1);
    sensor->moveWheel(third * revolutions, wheelCallback,2);
    sensor->moveWheel(fourth * revolutions, wheelCallback,3);
}


//for stopping
void stopWheels(){
    moving = false;
    sensor->stop(0);
    sensor->stop(1);
    sensor->stop(2);
    sensor->stop(3);
}

//turning the bot using the rotation of the wheels
void turnBot(float degrees){
    float startingZ = sensor->getRotation().z; //The initial rotation
    moving = true;
    sensor->turnWheel(90, wheelCallback,0);
    sensor->turnWheel(-90,wheelCallback,2);
    while(moving){} //Wait until wheel turning done
    float finishingZ = sensor->getRotation().z + degrees; //The final Rotation we want
    float currentDirection = degrees/std::abs(degrees); //Gets either 1 or -1
    float rotationDiff = std::abs(dist(finishingZ,startingZ,360)); //This gets our absolute difference
    powerWheels(200*currentDirection,1,-1,1,1);//Turns on the wheels to go for a while
    while(rotationDiff > 5){ //This goes until we are within 5 degrees of our initial tolerance
        usleep(2); //Sleeps for 2 milliseconds
        rotationDiff = std::abs(dist(finishingZ,sensor->getRotation().z,360)); //Checking our current situation
    }
    stopWheels();
    moving = true;
    sensor->turnWheel(-90, wheelCallback,0); //Turning back to normal
    sensor->turnWheel(90, wheelCallback,2); //Turning back to normal
    while(moving){}
}

//If we can't use wheel Rotation take the stuff from other turn except the rotating wheels
void tankTurn(float degrees){
    powerWheels(degrees,1,-1,-1,1);
    while(moving){}
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

//This is running in a separate thread and should be checking gyro every pit
void gyroControl(){
    
    float xThresh = 10.0;
    float yThresh = 10.0;
    while(true) {
        Vector3 gyro = sensor->getRotation();
        if(std::abs(gyro.x - initialX) >= xThresh) {
            gotBumped();
        }
        if(std::abs(gyro.y - initialY) >= yThresh) {
            gotBumped();
        }
    }
   
}

//This is called anytime we get a bump and its going to go backwards a fourth of a meter
void gotBumped(){
    avoiding = true;
    stopWheels();
    obstacleInFront = true;
    powerWheels(perMeter/4,-1,-1,-1,-1);
    while(moving);
    avoidObstacle();
}
//Call when want to Scan will wait until done scanning
void doScan(){
    scanning = true;
    sensor->scan(checkObstacle);
    while(scanning){} //wait till done scanning
}

//write obstacle avoidence stuff here when entered there is an obstacle in front
void avoidObstacle(){
    turnBot(-90*direction); //Should turn 90 degrees
    obstacleInFront = false;
    doScan();
    if(obstacleInFront) {
        turnBot(180*direction); //if there was an obstacle we need to check other side
        obstacleInFront = false;
        doScan();
        if (obstacleInFront) {//If we get here it means we are boxed in and need to back up
            turnBot(-90*direction);
            powerWheels(perMeter / 4, -1, -1, -1, -1);
            avoidObstacle(); //recall and try to go from there
        }
        powerWheels(perMeter/2,1,1,1,1);
        turnBot(-90*direction); //Turn back to forward
    }
    powerWheels(perMeter/2,1,1,1,1);
    direction = -direction; //flipDirection
    turnBot(-90*direction); //Turn back to forward
    avoiding = false;
}
//This should adjust to Beacon test a little to make sure its not turning the opposite direction
void adjustToBeacon(){
    float currentBeaconHeading = sensor->getHeading();
    float currentBotHeading = sensor->getRotation().z; //This gets our Bot Heading
    float degreesToAdjust = currentBeaconHeading-currentBotHeading;
    turnBot(degreesToAdjust);
}



//Navigation algorithm
void navigate() {
    doScan();
    //Once we done scanning check if theres an obstacle
    if (obstacleInFront) {
        avoidObstacle();
    }
    else {
        //we want to move half a meter
        powerWheels(perMeter / 2, 1, 1, 1, 1);
        while(moving);
    }


}

//int main(){
//    setup();
//    std::thread sensorThread(gyroControl);// set up thread for bumpers and gyros we want this always going
//    // while(!atBeacon){
//    //     if(!avoiding){ //if we aren't currently moving run our navigation again
//    //         adjustToBeacon();
//    //         navigate();
//    //     }
//    // }
//    powerWheels(perMeter / 2, 1, 1, 1, 1);
//    while(moving);
//
//
//}

#endif