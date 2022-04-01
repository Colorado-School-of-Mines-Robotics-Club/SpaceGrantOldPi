//
// Created by colin on 2/17/2022.
//

#include "robotControl.h"

//

int rangeFinderRadius = 10; //This is the largest distance the Rangefinder can see
int MapSize = 21; //This is the dimensions of the map and should be odd bigger dimensions more precise
std::vector<std::vector<int>> Map(MapSize, std::vector<int>(MapSize,0)); //This is the initial Map
bool obstacleInFront = false;
int obstacleProblemThreshold = 10;//How many points we need to see in a block for it to be an obstacle we want to avoid


//figures out motor commands to go towards beacon
void maintain() {

}

//does a general sensor check
void sensorCheck() {
    askBump();
    askGyro();
    sensor->scan(testObstacle);

}

//uses I2C and if something is wrong gets a vector with values and passes to problem
void askGyro() {
    std::vector<int> tester;
    problemGyro(tester);
}
//This will be called if there is an issue and send commands to motors to avoid
void problemGyro(std::vector<int> problem) {

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
void askLaser(std::vector<RangeFinderPacket>& packets){
    //Put a call to your function here
}
//adjusts path to avoid obstacle shown by clumps
void problemLaser(std::vector<int> clumps) {
    //solve laser problem
}


void setup() {
    //send signal to arduinos for calibration
    //swivel rangefinder
    //calibrate gyro
    //check beacon heading
}

//This should be sent into the Sensor-> scan and since it needs to be void it will set the global obstacleInFront
//to whether there is an obstacle
void testObstacle(std::vector<RangeFinderPacket> obstacles) {
    //Resetting Map
    for(int i = 0; i < MapSize; i++){
        for(int j = 0; j < MapSize; j++){
            Map[i][j] = 0;
        }
    }
    obstacleInFront = false;
    int botX = int(MapSize/2);
    int botY = int(MapSize/2);
    //Updating Map to reflect the scan
    for(auto & point : obstacles){
        int yOffset = round(sin(point.angle)*MapSize/rangeFinderRadius); //Calculates distance from bot in terms
        int xOffset = round(cos(point.angle)*MapSize/rangeFinderRadius); //Of indices
        Map[botY+yOffset][botX+xOffset] += 1; //Registers there is a obstacle at this location
    }
    for(int row = 0; row < MapSize; row++){
        for(int col = 0; col < MapSize; col++){
            if(Map[row][col] > obstacleProblemThreshold){
                obstacleInFront = true;
            }
        }
    }

}