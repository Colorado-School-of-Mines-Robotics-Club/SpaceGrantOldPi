cmake_minimum_required(VERSION 3.17)
project(SpaceGrantOldPi)

set(CMAKE_CXX_STANDARD 14)

add_executable(SpaceGrantOldPi PiFiles/BotCode/robotControl.cpp PiFiles/BotCode/robotControl.h)