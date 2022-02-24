//
// Created by mmurr on 2/18/2022.
//

#ifndef TESTER_TESTSENSORLIB_H
#define TESTER_TESTSENSORLIB_H

#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <functional>
#include <cmath>
#include <bitset>
#include <functional>
#include <nlohmann/json.hpp>
#include <fstream>
#include <thread>

#ifndef SENSOR_NAME
#define SENSOR_NAME sensor
#endif

#define SCANNER_HEIGHT 0.175
#define SCANNER_ANGLE 4

#define SCAN_POINTS 10
#define SCAN_ANGLE 90

#define INT_STATE_NONE 0
#define INT_STATE_GET_ANGLE 1
#define INT_STATE_SCAN 2

#define INT_STATE 1

#define WHEEL1 Wheel1
#define WHEEL2 Wheel2
#define WHEEL3 Wheel3
#define WHEEL4 Wheel4

#define ERROR_CONNECTION -1
#define ERROR_BUSY -2

#define SUCCESS 1
#define FAIL 0

struct RangeFinderPacket{
    float angle;
    uint16_t distance;
}__attribute__((packed));

struct Vector3{
    float x, y, z;
};

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
    int8_t scan(std::function<void(std::vector<RangeFinderPacket>&)> callbackFcn);


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
    int8_t getAngle(float angle, std::function<void(RangeFinderPacket&)> callbackFcn);

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

    // Internal function
    // Handles interrupt response from arduino
    void intHandler(int gpio, int level, uint32_t tick);

    // Gets the current gyro data
    // Uses a right hand coordinate system
    //  - x points forward
    //  - y points left
    //  - z points up
    //      use the right hand rule to translate these directions to angles
    //
    // Arguments:
    // 	none
    //
    // Returns Vector3 of current angle values
    Vector3 getGyro();

    // Sets the function that will be run when the gyro detects the bot is tilted too far
    //
    // Arguments:
    // 	callback -
    // 		function that will be called to alert the main program
    //      this takes an argument of a Vector3 which will be the current gyroscope data as of the alert
    // 	angle -
    // 		the minimum angle to trigger the alert at
    //
    // No return value
    void setGyroAlertFunction(std::function<void(Vector3)> callback, float angle);


    std::function<void(Vector3)> gyroAlertCallback;
    float gyroAlertAngle;

    double scannerAngle = -999;
    double scannerDistance = SCANNER_HEIGHT/std::tan(SCANNER_ANGLE*M_PI/180);
private:
    uint8_t _interruptState = 0;

    std::function<void(std::vector<RangeFinderPacket>&)> _scanCallback;

    std::function<void(RangeFinderPacket&)> _angleCallback;
    void constructorUni();

};





class Wheel{
public:


    Wheel(uint8_t address, uint8_t bus, uint8_t interruptPin);
    ~Wheel();

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
    int8_t turnWheel(float degrees, const std::function<void(int8_t)>& callback);


    // Turns the wheel until it reaches the limit switch
    // Arguments:
    //	callback -
    //		function that will be run when the wheel has completed or failed the operation
    //		this function should take an argument of type int8_t
    //		This argument is the response code from the arduino. >0 means success, otherwise failed
    //		Currently this will never actually be a fail but I wanted to keep the support
    //
    // Returns 0 if sucessful - othewise ERROR_BUSY
    int8_t resetRotation(std::function<void(int8_t)> callback);



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
    int8_t setRotation(float degrees, std::function<void(int8_t)> callback);


    // Returns the current rotation of the wheel in degrees
    // Try to limit how many times this is called or run it asyncronously as I2C is a relatively slow interface
    float getRotation() const;



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
    //
    // Returns 0 if sucessful - othewise ERROR_BUSY
    int8_t move(float revolutions, const std::function<void(int8_t)>& callback);


    // Turns on the drive motor
    // The motor will not stop until stop() or move() are called
    void drive();


    // Stops the drive motor
    // Designed to be used with drive but will also stop move()
    void stop();


    // Returns the current position of the drive motor in revolutions
    // Try to limit how many times this is called or run it asyncronously as I2C is a relatively slow interface
    float getPosition();

    // Sets the function that will be called whenever the push sensor on this wheel touches something
    void setPressureAlertFunction(std::function<void(uint8_t)> callback);

    // Gets the current value of this wheels push sensor
    bool getPressureSensor() const;




    // Internal functions
    // intHandler handles any interrupts caused by this wheel
    // intCallback saves the callback function
    // intHandler basically just calls intCallback
    void intHandler(int gpio, int level, uint32_t tick);
    std::function<void(int8_t)> intCallback;
    int8_t running = 0;
    int8_t turning = 0;
    double position = 0;
    double rotation = 0;

    bool pressurePressed = false;

    std::function<void(uint8_t)> pressureAlertCallback;

    // Saves what the wheel is currently doing
    uint8_t _state = INT_STATE_NONE;
    uint8_t _address;

private:
    // I2C information
    uint8_t _bus;



    // Helper functions for I2C
    // writeData writes data to register reg
    // writeRegister will write data to register reg or just sends reg
    // readData reads data from a register and returns a pointer to the data
    void writeData(uint8_t reg, void* data, uint8_t length);
    void writeRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t data);
    void* readData(uint8_t reg, uint8_t length);

    void* thisWheel;

};


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

extern Sensor* sensor;
extern bool drawing;

extern Wheel* Wheel1;
extern Wheel* Wheel2;
extern Wheel* Wheel3;
extern Wheel* Wheel4;

#endif //TESTER_TESTSENSORLIB_H
