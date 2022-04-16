//
// Created by Shane on 4/15/2022.
//
#ifndef remoteControl_cpp
#define remoteControl_cpp
#include <iostream>
#include <unistd.h>  /* only for sleep() */

#include "robotControl.h"

using namespace std;


void rMove(int direction) {
    powerWheels(0.25 * direction,1,1,1,1);
}

void rTurn(int direction) {
    powerWheels(0.1,1 * direction, 1 * direction, -1 * direction, -1  * direction);
}

int chooseAction(char cmd) {
    switch(cmd) {
        case 'a':
            rTurn(1);
            break;
        case 'd':
            rTurn(-1);
            break;
        case 'w':
            rMove(1);
            break;
        case 's':
            rMove(-1);
            break;
        case 'e':
            return 1;
            break;
    }
    return 0;
}

int main(void)
{

    setup();
    char cmd;
    while(true) {
        cout << "wasd to drive; e to exit" << endl;
        cin >> cmd;
        cout << cmd << endl;
        if(chooseAction(cmd)) {
            exit(0);
        }
    }
}

#endif