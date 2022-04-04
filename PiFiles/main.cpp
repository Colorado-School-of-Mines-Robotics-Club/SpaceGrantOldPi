#include "BotCode/SensorLib.h"
#include <iostream>
#include <unistd.h>
#include "BotCode/robotControl.h"

bool ready = true;

void printPacket(RangeFinderPacket& packet){
	std::cout << "Angle: " << packet.angle << std::endl;
	std::cout << "Distance: " << packet.distance << std::endl;
	ready = true;
}

void printPackets(std::vector<RangeFinderPacket>& packets){
	for(unsigned int i = 0; i < packets.size(); i++){
		std::cout << "Packet " << i << ":" << std::endl;
		printPacket(packets[i]);
	}
	ready = true;
}

void printResponse(int8_t response){
	std::cout << "Response: " << (int)response << std::endl;
	ready = true;
}

int main(){
	// General gpio initialization
	if(gpioInitialise() < 0){
		std::cout << "GPIO failed to initialize!" << std::endl;
		return -1;
	}else{
		std::cout << "GPIO initialized!" << std::endl;
	}

	// Initialize the Sensor object
	// The name of the variable can be set by defining SENSOR_NAME before including SensorLib.h - by default its just 'sensor'
	// Yes I hate this solution too
	//sensor = new Sensor();

	/*while(true){
		std::this_thread::sleep_for(std::chrono::seconds(1));
		Vector3 test = sensor->getRotation();
		std::cout << "X: " << test.x << "\tY: " << test.y << "\tZ: " << test.z << std::endl;

	}*/

	// Also init wheel
	Wheel1 = new Wheel(0x1A, 1, 17);
	Wheel1->setPressureAlertFunction([](){std::cout << "Pressure Pressed" << std::endl;});

	// Get distance from angle 50 degrees and print the result
	/*std::cout << "Sending Angle Command..." << std::endl;
	sensor->getAngle(50.0, printPacket);
	ready = false;

	// Do other stuff here while arduino responds
	while(!ready);

	// Get distance from angle -20.5 degrees and print the result
	std::cout << "Sending Angle Command..." << std::endl;
	sensor->getAngle(-20.5, printPacket);
	ready = false;

	// Do other stuff here while arduino responds
	while(!ready);


	// Scan the full range of the sensor and print the results
	std::cout << "Sending Scan Command..." << std::endl;
	ready = false;

	sensor->scan(printPackets);
	
	// Do other stuff here while arduino responds
	while(!ready);

	// Get heading and rssi and print it
	// Note this command is blocking - may want to run it asyncronously
	std::cout << "Getting Radio Data..." << std::endl;
	float heading;
	uint8_t rssi;
	sensor->getHeadingRSSI(heading, rssi);
	std::cout << "Heading: " << heading << ", RSSI: " << (int)rssi << std::endl;*/

	// #################################
	// Wheel stuff
	
	/*std::cout << std::endl << "Testing Wheel..." << std::endl;
	std::cout << "Turning 45 degrees..." << std::endl;
	ready = false;

	Wheel1->turnWheel(130, printResponse);

	while(!ready);

	std::cout << "Setting rotation to -45 degrees..." << std::endl;
	ready = false;
	Wheel1->setRotation(-130, printResponse);

	while(!ready);

	std::cout << "Current rotation: " << Wheel1->getRotation() << std::endl;

	std::cout << "Resetting rotation..." << std::endl;
	ready = false;
	Wheel1->resetRotation(printResponse);

	while(!ready);

	std::cout << "Moving wheel 0.5 rotations..." << std::endl;
	ready = false;
	Wheel1->move(0.2, printResponse);

	while(!ready);

	std::cout << "Starting motor..." << std::endl;
	Wheel1->drive();

	sleep(1);

	std::cout << "Stopping motor..." << std::endl;
	Wheel1->stop();

	std::cout << "Current motor position: " << Wheel1->getPosition() << std::endl;*/


	bool exit = true;
	while(exit){
		std::cout << "Waiting for command: ";
		char c;
		std::cin >> c;

		switch(c){
		case 'r':{
			float input;
			std::cout << "Rotate degrees: ";
			std::cin >> input;
			std::cout << std::endl;

			Wheel1->turnWheel(input, printResponse);
			break;
		}
		
		case 's':{
			float input;
			std::cout << "Set rotation: ";
			std::cin >> input;
			std::cout << std::endl;

			Wheel1->setRotation(input, printResponse);
			break;
		}

		case 'R':{
			std::cout << "Resetting rotation..." << std::endl;

			Wheel1->resetRotation(printResponse);
			break;
		}

		case 'm':{
			float input;
			std::cout << "Move wheel: ";
			std::cin >> input;
			std::cout << std::endl;

			Wheel1->move(input, printResponse);
			break;
		}

		case 'D':{
			std::cout << "Driving..." << std::endl;
			Wheel1->drive();
			break;
		}

		case 'd':{
			std::cout << "Stopping..." << std::endl;
			Wheel1->stop();
			break;
		}

		case 'w':{
			std::cout << "Wheel position: " << Wheel1->getPosition() << std::endl;
			break;
		}

		case 't':{
			std::cout << "Turn position: " << Wheel1->getRotation() << std::endl;
			break;
		}

		case 'e':{
			exit = false;
			break;
		}

		case '?':{
			std::cout << std::endl << "-------------------------------" << std::endl;
			std::cout << "r - rotate wheel" << std::endl;
			std::cout << "s - set rotation" << std::endl;
			std::cout << "R - reset rotation" << std::endl;
			std::cout << "m - move wheel" << std::endl;
			std::cout << "D - drive" << std::endl;
			std::cout << "d - stop" << std::endl;
			std::cout << "w - get drive position" << std::endl;
			std::cout << "t - get turn position" << std::endl;
			std::cout << "e - exit" << std::endl;
			std::cout << "-------------------------------" << std::endl << std::endl;
			break;
		}

		default:{
			std::cout << "Unknown command" << std::endl;
		}

		}
	}
	return 0;
}

