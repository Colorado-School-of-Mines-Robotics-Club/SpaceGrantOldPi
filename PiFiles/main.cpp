#include "SensorLib.h"
#include <iostream>
#include <unistd.h>
#include "robotControl.h"

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
	sensor = new Sensor();

	while(true){
		std::this_thread::sleep_for(std::chrono::seconds(1));
		Vector3 test = sensor->getRotation();
		std::cout << "X: " << test.x << "\tY: " << test.y << "\tZ: " << test.z << std::endl;

	}

	// Also init wheel
	/*Wheel1 = new Wheel(0x1A, 1, 17);

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
	std::cout << "Heading: " << heading << ", RSSI: " << (int)rssi << std::endl;

	// #################################
	// Wheel stuff
	
	std::cout << std::endl << "Testing Wheel..." << std::endl;
	std::cout << "Turning 45 degrees..." << std::endl;
	ready = false;

	Wheel1->turnWheel(45, printResponse);

	while(!ready);

	std::cout << "Setting rotation to -45 degrees..." << std::endl;
	ready = false;
	Wheel1->setRotation(-45, printResponse);

	while(!ready);

	std::cout << "Current rotation: " << Wheel1->getRotation() << std::endl;

	std::cout << "Resetting rotation..." << std::endl;
	ready = false;
	Wheel1->resetRotation(printResponse);

	while(!ready);

	std::cout << "Moving wheel 0.5 rotations..." << std::endl;
	ready = false;
	Wheel1->move(0.5, printResponse);

	while(!ready);

	std::cout << "Starting motor..." << std::endl;
	Wheel1->drive();

	sleep(5);

	std::cout << "Stopping motor..." << std::endl;
	Wheel1->stop();

	std::cout << "Current motor position: " << Wheel1->getPosition() << std::endl;
	*/
	return 0;
}

