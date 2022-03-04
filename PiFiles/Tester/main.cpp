#include <iostream>
#include "TestSensorLib.h"

void alertFnc(Vector3 in){
    std::cout << "On incline - X: " << in.x << ", Y: " << in.y << ", Z: " << in.z << std::endl;
}

void scanResponse(std::vector<RangeFinderPacket>& in){
    std::cout << "Distances:" << std::endl;
    for(auto & packet : in){
        std::cout << "\tDistance: " << (int)packet.distance << ", Angle: " << packet.angle << std::endl;
    }
    std::cout << std::endl;
}

void pressureAlert(uint8_t wheel){
    std::cout << "Wheel at address " << (int)wheel << " has touched something." << std::endl;
}

int main() {
    // Init sensor and set alert functions
    sensor = new Sensor();
    sensor->setGyroAlertFunction(alertFnc, 5);
    Wheel1->setPressureAlertFunction(pressureAlert);
    Wheel2->setPressureAlertFunction(pressureAlert);
    Wheel3->setPressureAlertFunction(pressureAlert);
    Wheel4->setPressureAlertFunction(pressureAlert);

    //sensor->scan(scanResponse);
    sensor->getAngle(5, [](RangeFinderPacket& a){std::cout << "Distance: " << (int)a.distance << ", Angle: " << a.angle << std::endl;});
    //Wheel1->setRotation(180, [](int8_t a){std::cout << "Stopped Turning" << std::endl;});
    //Wheel2->setRotation(90, [](int8_t a){std::cout << "Stopped Turning" << std::endl;});
    //Wheel3->setRotation(0, [](int8_t a){std::cout << "Stopped Turning" << std::endl;});
    //Wheel4->setRotation(-90, [](int8_t a){std::cout << "Stopped Turning" << std::endl;});

    std::this_thread::sleep_for(std::chrono::seconds(2));

    Wheel1->move(4, [](int8_t a){std::cout << "Stopped 1" << std::endl;});
    Wheel2->move(4, [](int8_t a){std::cout << "Stopped 1" << std::endl;});
    Wheel3->move(4, [](int8_t a){std::cout << "Stopped 1" << std::endl;});
    Wheel4->move(4, [](int8_t a){std::cout << "Stopped 1" << std::endl;});

    std::this_thread::sleep_for(std::chrono::seconds(16));

    sensor->scan(scanResponse);

    std::this_thread::sleep_for(std::chrono::seconds(10));

    Wheel1->drive();
    Wheel2->drive();
    Wheel3->drive();
    Wheel4->drive();

    std::this_thread::sleep_for(std::chrono::seconds(40));


    Wheel1->stop();
    Wheel2->stop();
    Wheel3->stop();
    Wheel4->stop();

    Wheel1->move(-5, [](int8_t a){std::cout << "Stopped 1" << std::endl;});
    Wheel2->move(-4, [](int8_t a){std::cout << "Stopped 2" << std::endl;});
    Wheel3->move(-3, [](int8_t a){std::cout << "Stopped 3" << std::endl;});
    Wheel4->move(-2, [](int8_t a){std::cout << "Stopped 4" << std::endl;});

    // drawing is set to false when window is closed - wait for that before exiting the program
    while(drawing);

    return 0;
}
