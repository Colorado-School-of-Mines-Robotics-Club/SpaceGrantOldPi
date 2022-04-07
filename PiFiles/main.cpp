#include "BotCode/SensorLib.h"
#include <iostream>
#include <unistd.h>
#include "BotCode/robotControl.h"

bool ready = true;

void printPacket(RangeFinderPacket& packet){
	std::cout << "\tAngle: " << packet.angle << std::endl;
	std::cout << "\tDistance: " << (int)packet.distance << std::endl;
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
	sensor = new Sensor();

	/*while(true){
		std::this_thread::sleep_for(std::chrono::seconds(1));
		Vector3 test = sensor->getRotation();
		std::cout << "X: " << test.x << "\tY: " << test.y << "\tZ: " << test.z << std::endl;

	}*/

	// Also init wheel
	//Wheel1 = new Wheel(0x1A, 1, 17);
	//Wheel1->setPressureAlertFunction([](){std::cout << "Pressure Pressed" << std::endl;});

	bool exit = true;
	while(exit){
		std::cout << "Waiting for command: ";
		char c;
		std::cin >> c;

		switch(c){
		/*case 'r':{
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
		}*/

		case 'e':{
			exit = false;
			break;
		}
			case 'S':{
				int points;
				float angle;
				std::cout << "Scan points: ";
				std::cin >> points;
				std::cout << "Scan angle: ";
				std::cin >> angle;
				std::cout << "Scanning..." << std::endl;
				if(sensor->scan(printPackets, points, angle) == ERROR_BUSY) std::cout << "Response: Busy" << std::endl;
				break;
			 }

			case 'A':{
				float input;
				std::cout << "Get angle: ";
				std::cin >> input;
				if(sensor->getAngle(input, printPackets) == ERROR_BUSY) std::cout << "Response: Busy" << std::endl;
				break;
			 }

			case 'H':{
				std::cout << "Heading: " << sensor->getHeading() << std::endl;
				break;
			 }

			case 'O':{
				Vector3 rotation = sensor->getRotation();
				std::cout << "Rotation:" << std::endl;
				std::cout << "\tX: " << rotation.x << std::endl;
				std::cout << "\tY: " << rotation.y << std::endl;
				std::cout << "\tZ: " << rotation.z << std::endl;
				break;

			 }



		case '?':{
			std::cout << std::endl << "-------------------------------" << std::endl;
			std::cout << "Wheel commands: " << std::endl;
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
			std::cout << "Sensor commands: " << std::endl;
			std::cout << "S - scan" << std::endl;
			std::cout << "A - get angle" << std::endl;
			std::cout << "H - get heading" << std::endl;
			std::cout << "O - get rotation" << std::endl;
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

