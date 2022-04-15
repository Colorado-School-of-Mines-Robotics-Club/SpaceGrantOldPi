//
// Created by Shane on 4/15/2022.
//

#include <ncurses.h>
#include <iostream>
#include <unistd.h>  /* only for sleep() */

#include "robotControl.cpp"
#include "SensorLib.cpp"


void rMove(int direction) {
    powerWheels(0.25 * direction,1,1,1,1);
    printw("Moving!\n");

}

void rTurn(int direction) {
    powerWheels(0.1,1 * direction, 1 * direction, -1 * direction, -1  * direction);
    printw("Turning!\n");
}

void chooseAction(int cmd) {
    switch(cmd) {
        case 97:
            rTurn(1);
            break;
        case 100:
            rTurn(-1);
            break;
        case 119:
            rMove(1);
            break;
        case 115:
            rMove(-1);
            break;
        case 48:
            endwin();
            _exit(0);
    }
}



int kbhit(void)
{
    int ch = getch();

    if (ch != ERR) {
        ungetch(ch);
        return 1;
    } else {
        return 0;
    }
}

int main(void)
{

    setup();
    initscr();

    cbreak();
    noecho();
    nodelay(stdscr, TRUE);

    scrollok(stdscr, TRUE);
    while (1) {
        if (kbhit()) {
            chooseAction(getch());
            refresh();
        } else {
            refresh();
            sleep(1);
        }
    }
}