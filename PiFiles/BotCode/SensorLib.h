#ifndef SENSORLIB_H
#define SENSORLIB_H

#include <pigpio.h>
#include <iostream>
#include <vector>
#include <functional>
#include <cmath>
#include <bitset>
#include <functional>
#include <chrono>
#include <thread>
#include <memory.h>
#include <unistd.h>
#include <fstream>
#include <string>

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#ifndef SENSOR_NAME
#define SENSOR_NAME sensor
#endif

#define SCAN_POINTS 10

#define INT_STATE_NONE 0
#define INT_STATE_GET_ANGLE 1
#define INT_STATE_SCAN 2

#define INT_STATE 1

#define HANDSHAKE_REGISTER 0x01
#define SCAN_REGISTER 0x02
#define GET_ANGLE_REGISTER 0x03
#define GET_HEADING_REGISTER 0x04
#define GET_RSSI_REGISTER 0x05
#define SCAN_RESPONSE_REGISTER 0x06
#define GET_ROTATION_REGISTER 0x07
#define POLL_REGISTER 0x08

#define TURN_REGISTER 0x09
#define RESET_ROTATION_REGISTER 0x0A
#define SET_ROTATION_REGISTER 0x0B
#define GET_WHEEL_ROTATION_REGISTER 0x0C
#define MOVE_REGISTER 0x0D
#define DRIVE_REGISTER 0x0E
#define STOP_REGISTER 0x0F
#define GET_POSITION_REGISTER 0x10
#define RESPONSE_REGISTER 0x11
#define PRESSURE_REGISTER 0x12


#define STATUS_SCAN_DONE 0b0001
#define STATUS_TURN_DONE(motor) (0b001 << (motor*3 + 1))
#define STATUS_DRIVE_DONE(motor) (0b010 << (motor*3 + 1))
#define STATUS_PUSH_BUTTON(motor) (0b100 << (motor*3 + 1))

#define WHEEL1 Wheel1
#define WHEEL2 Wheel2
#define WHEEL3 Wheel3
#define WHEEL4 Wheel4

#define ERROR_CONNECTION -1
#define ERROR_BUSY -2

#define SUCCESS 1
#define FAIL 0

#define BAUDRATE 32000
#define SPICHAN 0
#define SPILENGTH(x) (x << 10)
#define SPIFLAGS (1 << 9)
#define POLL_REGISTER 0x08

#define SENSOR_INT_PIN 18



struct RangeFinderPacket{
	float angle;
	uint16_t distance;
}__attribute__((packed));

struct Vector3{
	float x, y, z = 0;
}__attribute__((packed));


class Sensor{
public:
	Sensor();
	Sensor(uint8_t address, uint8_t bus);
	~Sensor();

	// Takes distance measurements at every point within the scan range
	// Arguments:
	// 	callbackFcn - 
	// 		Function which is called when the arduino responds with data.
	// 		callbackFcn should have no return value and should take a vector of RangeFinderPacket values as an argument
	// 		The vector will contain all the data returned from the arduino	
	//
	// Returns 0 if sucessful - othewise a negative error code
	int8_t scan(std::function<void(std::vector<RangeFinderPacket>&)> callbackFcn, uint8_t points = 10, float angle = 50);

	
	// Gets distance measurement from a specified angle
	// Arguments:
	// 	angle -
	// 		float which specifies the angle to get a measurement from
	//
	// 	callbackFcn -
	// 		Function which is called when the arduino responds with data
	// 		callbackFcn should have no return value and should take a RangeFinderPacket value as an argument
	// 		The RangeFinderPacket argument contains the data returned from the arduino
	//
	// Returns 0 if sucessful - otherwise a negative error code
	int8_t getAngle(float angle, std::function<void(std::vector<RangeFinderPacket>&)> callbackFcn);

	// Gets the angle repored by the beacon with the lowest RSSI value
	// Note: This function is blocking and must wait for the relatively slow I2C communication
	// 	Its reccomended to run this asyncronously
	//
	// Arguments: none
	//
	// Returns the angle as a float
	float getHeading();

	// Gets the lowest RSSI value from the beacon
	// Note: This function is blocking and must wait for the relatively slow I2C communication
	// 	Its reccomended to run this asyncronously
	//
	// Arguments: none
	//
	// Returns the RSSI value as an unsigned 8 bit integer
	uint8_t getRSSI();

	// Gets the angle and RSSI value from the lowest reported rssi from the beacon
	// Note: This function is blocking and must wait for the relatively slow I2C communication
	// 	Its reccomended to run this asyncronously
	//
	// Arguments:
	// 	heading -
	// 		float to contain the heading value
	// 	rssi -
	// 		byte to contain the rssi value
	//
	// No return value - all values are set via pass by reference
	void getHeadingRSSI(float& heading, uint8_t& rssi);

	// Gets the current rotation of the bot as a Vector3
	// Vector 3 has 3 properties - x, y, and z
	// Uses a right hand coordinate system
	//		
	//		Looking at arduino from top view:
	//		/---------------------------------------\
	//		|										|
	//		|(usb port)			^ y					|
	//		|					|					|
	//		|			   z(up)O---> x				|
	//		|										|
	//		|(9v port)										|
	//		|										|
	//		\---------------------------------------/
	//
	// Use right hand rule to get rotation from these coordinate axes
	Vector3 getRotation();

	void intHandler(int pin, int level, uint32_t tick);

	//Helper function for int handler to handle any scans
	void handleScan(int pin, int level, uint32_t tick);


	//WHEEL FUNCTIONS go below here


	//
    // Moves the wheel a specified number of revolutions
    // Arguments:
    //	revolutions -
    //		Number of revolutions to move
    //
    //	callback -
    //		function that will be run when the wheel has completed or failed the operation
    //		this function should take an argument oftype int8_t
    //		This argument is the response code from the arduino. >0 means success, otherwise failed
    //		Currently this will never actually be a fail but I wanted to keep the support
    //  Wheel -
    //      this is which wheel we are moving
    //      Wheel int should be 0,1,2,3
    //
    // Returns 0 if sucessful - othewise ERROR_BUSY
	void moveWheel(float revolutions, std::function<void(int8_t)> callback, int Wheel);

    // Turns the wheel a specified amount of degrees
    // Arguments:
    //	degrees -
    //		Amount of degrees to turn the wheel
    //
    //	callback -
    //		function that will be run when the wheel has completed or failed the operation
    //		this function should take an argument of type int8_t
    //		This argument is the response code from the arduino. >0 means success, otherwise failed
    //		Currently this will never actually be a fail but I wanted to keep the support
    //
    // Returns 0 if sucessful - othewise ERROR_BUSY
    void turnWheel(float degrees, std::function<void(int8_t)> callback, int8_t Wheel);

    // Sets the callback function which will be run when this wheel's push sensor hits something
    // Arguments:
    // 	callback -
    // 		The function that will be run upon detecting an object
    //
    // No return value
    void setPressureAlertFunction(std::function<void()> callback);


    // Turns on the drive motor
    // The motor will not stop until stop() or move() are called
    void drive(int8_t Wheel);

    // Stops the drive motor
    // Designed to be used with drive but will also stop move()
    void stop(int8_t Wheel);


    // Turns the wheel until it reaches the limit switch
    // Arguments:
    //	callback -
    //		function that will be run when the wheel has completed or failed the operation
    //		this function should take an argument of type int8_t
    //		This argument is the response code from the arduino. >0 means success, otherwise failed
    //		Currently this will never actually be a fail but I wanted to keep the support
    //  wheel -
    //      this is the wheel we are resetting
    //
    // Returns 0 if sucessful - othewise ERROR_BUSY
    int8_t resetRotation(std::function<void(int8_t)> callback, int8_t Wheel);


    // Sets the absolute rotation of the wheel
    // Arguments:
    //	degrees -
    //		Amount of degrees to turn the wheel
    //
    //	callback -
    //		function that will be run when the wheel has completed or failed the operation
    //		this function should take an argument of type int8_t
    //		This argument is the response code from the arduino. >0 means success, otherwise failed
    //		Currently this will never actually be a fail but I wanted to keep the support
    //
    // Returns 0 if sucessful - othewise ERROR_BUSY
    int8_t setRotation(float degrees, std::function<void(int8_t)> callback, int8_t Wheel);




private:
	uint8_t _addr;
	int _fd;
	uint8_t _bus;

	std::function<void(int8_t)> _turnIntCallback;
	std::function<void(int8_t)> _driveIntCallback;
	std::function<void()> _pushIntCallback;

	std::fstream arduino;
	int arduinoPort;
	
	uint8_t readRegister(uint8_t reg);
	void readRegister(uint8_t reg, char* buffer, uint8_t length);
	uint8_t writeRegister(uint8_t reg, char* buffer, uint8_t length);

	std::function<void(std::vector<RangeFinderPacket>&)> _callback;
	void constructorUni();

	int dataRequired = 0;
	std::vector<RangeFinderPacket> responseData;
	float scanAngle;

};

	
	
	
	
//class Wheel {
//public:
//
//
//    Wheel(uint8_t address, uint8_t bus, uint8_t interruptPin);
//
//    ~Wheel();
//
//    uint8_t interruptPin;
//
//
//
//
//
//
//
//    // Returns the current rotation of the wheel in degrees
//    // Try to limit how many times this is called or run it asyncronously as I2C is a relatively slow interface
//    float getRotation();
//
//
//    // Returns the current position of the drive motor in revolutions
//    // Try to limit how many times this is called or run it asyncronously as I2C is a relatively slow interface
//    float getPosition();
//
//    bool getPressureSensor();
//
//    // Internal functions
//    // intHandler handles any interrupts caused by this wheel
//    // intHandler basically just calls intCallback
//
//};


// This part declares the sensor object to be used and is the interrupt function
// The name of the sensor object can be changed by defining SENSOR_NAME before including this file
// 
// pigpio's function to set interrupts requires you to pass the function to be called as an argument
// The problem is member functions cannot be passed as an argument in C++
// (No lambdas which capture the current instance of the Sensor class do not work either)
// But any interrupt handler would need access to variables within its instance of the Sensor class (such as the callback function the user provided or how to handle the interrupt)
// 
// Because of this, this solution creates a pointer to the Sensor object immediately without initializing it (must be a pointer otherwise the constructor will be called which would cause other issues)
// The interrupt function is used as a global function that is passed as the pigpio interrupt callback. It calls the interrupt handler of the sensor object depending on what gpio pin was triggered
//
// This is fine because in order for the interrupt function to be called, the sensor object must have been initialized by the user so we shouldnt get a nullptr error
// We just need to make sure the deconstructor of Sensor detaches this function from the interrupt or we will get a nullptr error
//
// Yes I hate this solution as much as everyone else
// Go ahead and fix it if you can
extern Sensor* SENSOR_NAME;

extern Wheel* WHEEL1;
extern Wheel* WHEEL2;
extern Wheel* WHEEL3;
extern Wheel* WHEEL4;

inline void interrupt(int gpio, int level, uint32_t tick){
	if(level != 1) return;
	if(gpio == SENSOR_INT_PIN){
		SENSOR_NAME->intHandler(gpio, level, tick);

	}
}
#endif

