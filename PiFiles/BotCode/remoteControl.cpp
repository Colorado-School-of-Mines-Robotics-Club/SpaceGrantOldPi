//
// Created by Shane on 4/15/2022.
//

#include <vector>
//#include "SensorLib.h"
//#include "robotControl.h"
#include <conio.h>



using namespace std;

void turn(int direction) {
//    powerWheels(0.1, 1 * direction, 1 * direction, -1 * direction, -1 * direction)
    cout << "Turning..." << endl;
};

void move(int direction) {
//    powerWheels(0.25 * direction,1,1,1,1);
    cout << "Moving..." << endl;
}


int main {
//    setup();
    char cmd;
    while(cmd != '0') {
        cmd = _getch();
        switch(cmd) {
            case 'w':
                move(1);
                break;
            case 's':
                move(-1);
                break;
            case 'a':
                turn(1);
                break;
            case 'd':
                turn(-1);
                break;
        }
    }

}