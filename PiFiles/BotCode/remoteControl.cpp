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
    powerWheels(0.5 * direction,1,1,1,1);
}

void bigMove(int direction) {
    powerWheels(2.0 * direction,1,1,1,1);
}

void rTurn(int direction) {
    powerWheels(0.5,-1 * direction, -1 * direction, 1 * direction, 1  * direction);
}

void bigTurn(int direction) {
    powerWheels(2.0,-1 * direction, -1 * direction, 1 * direction, 1  * direction);
}

bool chooseAction(char cmd) {
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
        case 'W':
            bigMove(1);
            break;
        case 'S':
            bigMove(-1);
            break;
        case 'A':
            bigTurn(1);
            break;
        case 'D':
            bigTurn(-1);
            break;
        case 'e':
            exit(0);
    }
    return false;
}

int main(void)
{

    setup();
    char cmd;
    while(true) {
        cout << "wasd/WASD to drive; e to exit" << endl;
        cin >> cmd;
        cout << cmd << endl;
        chooseAction(cmd);
        cout << "Action done." << endl;
    }
}

#endif