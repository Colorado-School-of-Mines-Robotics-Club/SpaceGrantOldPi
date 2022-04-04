#include "SensorLib.h"

Sensor* SENSOR_NAME;

Wheel* WHEEL1;
Wheel* WHEEL2;
Wheel* WHEEL3;
Wheel* WHEEL4;

char SERTTY[] = "/dev/ttyS0";

Sensor::Sensor(uint8_t address, uint8_t bus){
	_addr = address;
	_bus = bus;
	constructorUni();
}

Sensor::Sensor(){
	_addr = 0x19;
	_bus = 1;
	constructorUni();
}

// Deconstructor removes the interrput to avoid nullptr errors
Sensor::~Sensor(){
}

void Sensor::constructorUni(){
	// Open I2C bus
	int8_t handle = serOpen(SERTTY, BAUDRATE, 0); 

	// Make sure connection has been established
	if(handle < 0){
		std::cerr << "Unable to connect to sensors." << std::endl;
		return;
	}
	
	// Read data
	serWriteByte(handle, HANDSHAKE_REGISTER);
	while(!serDataAvailable(handle));
	uint8_t data = serReadByte(handle);

	// Close I2C bus
	serClose(handle);

	// Make sure data is correct
	if(data != _addr){
		std::cerr << "Sensor handshake failed" << std::endl;
		std::cerr << "Expected: " << (int)_addr << std::endl;
		std::cerr << "Got:      " << (int)data << std::endl;
		return;
	}


	// Tell user everything is ok
	std::cout << "Succesfully connected to sensors!" << std::endl;
}


Vector3 Sensor::getRotation(){
	// Open I2C bus
	int8_t handle = serOpen(SERTTY, BAUDRATE, 0); 

	Vector3 output;

	// Make sure connection has been established
	if(handle < 0){
		std::cerr << "Unable to connect to sensors." << std::endl;
		return output;
	}

	serWriteByte(handle, SENSOR_GET_ROTATION_REGISTER);
	while(!serDataAvailable(handle));
	serRead(handle, (char*)&output, sizeof(output)); 

	serClose(handle);

	return output;
}

int8_t Sensor::getAngle(float angle, std::function<void(RangeFinderPacket&)> callbackFcn){
	// Open I2C bus
	int8_t handle = serOpen(SERTTY, BAUDRATE, 0); 

	// Make sure connection is established
	if(handle < 0){
		std::cerr << "Unable to connect to sensor" << std::endl;
		return ERROR_CONNECTION;
	}

	char buffer[sizeof(angle) + 1];
	*(float*)(buffer + 1) = angle;
	*(uint8_t*)buffer = GET_ANGLE_REGISTER;

	// Send command
	serWrite(handle, buffer, sizeof(angle) + 1);
	while(!serDataAvailable(handle));
	uint8_t response = serReadByte(handle);

	// Close I2C transmission
	serClose(handle);


	if(response){
		// Busy
		return ERROR_BUSY;
	}

	if(!fork()){
		int8_t handle = serOpen(SERTTY, BAUDRATE, 0);
		
		if(handle < 0){
			std::cerr << "Unable to connect to sensor" << std::endl;
			exit(0);
		}
		while(true){
			usleep(10000);

			if(!serDataAvailable(handle)){
				continue;
		 	}		
			std::cout << "Responded" << std::endl;

			RangeFinderPacket response;
			serRead(handle, (char*)&response, sizeof(response));

			serClose(handle);

			callbackFcn(response);

			exit(0);
		}
	}

	return 0;
}

int8_t Sensor::scan(std::function<void(std::vector<RangeFinderPacket>&)> callbackFcn, uint8_t points, float angle){
	// Open I2C bus
	int8_t handle = serOpen(SERTTY, BAUDRATE, 0); 

	// Make sure connection is established
	if(handle < 0){
		std::cerr << "Unable to connect to sensor" << std::endl;
		return ERROR_CONNECTION;
	}

	char buffer[sizeof(points) + sizeof(angle) + 1];
	*(uint8_t*)buffer = SCAN_REGISTER;
	*(uint8_t*)(buffer + 1) = points;
	*(float*)(buffer + 2) = angle;

	// Send command
	serWrite(handle, buffer, sizeof(points) + sizeof(angle) + 1);
	while(!serDataAvailable(handle));
	uint8_t response = serReadByte(handle);

	// Close I2C transmission
	serClose(handle);

	if(response){
		// Busy
		return ERROR_BUSY;
	}


	if(!fork()){
		while(true){
			usleep(10000);
			int8_t handle = serOpen(SERTTY, BAUDRATE, 0);

			if(handle < 0){
				std::cerr << "Unable to connect to sensor" << std::endl;
				continue;
			}

			if(!serDataAvailable(handle)){
				serClose(handle);
				continue;
		 	}		
			uint8_t length = serDataAvailable(handle);

			char response[length];
			serRead(handle, response, length);

			serClose(handle);

			std::vector<RangeFinderPacket> output(length/sizeof(RangeFinderPacket));

			for(uint8_t i = 0; i < length/sizeof(RangeFinderPacket); i++){
				output[i] = *(RangeFinderPacket*)(response + i*sizeof(RangeFinderPacket));
			}

			callbackFcn(output);

			exit(0);
		}
	}

	return 0;

}


float Sensor::getHeading(){
	float output;
	int8_t handle = serOpen(SERTTY, BAUDRATE, 0);

	if(handle < 0){
		std::cerr << "Unable to connect to sensor" << std::endl;
		return 0;
	}

	serWriteByte(handle, GET_HEADING_REGISTER);
	while(!serDataAvailable(handle));
	serRead(handle, (char*)&output, sizeof(output));

	serClose(handle);

	return output;
}

uint8_t Sensor::getRSSI(){
	uint8_t output;
	int8_t handle = serOpen(SERTTY, BAUDRATE, 0);

	if(handle < 0){
		std::cerr << "Unable to connect to sensor" << std::endl;
		return 255;
	}

	serWriteBute(handle, GET_RSSI_REGISTER);
	while(!serDataAvailable(handle));
	output = serReadByte(handle);

	serClose(handle);

	return output;
}

void Sensor::getHeadingRSSI(float& heading, uint8_t& rssi){
	heading = getHeading();
	rssi = getRSSI();
}





Wheel::Wheel(uint8_t address, uint8_t bus, uint8_t interruptPin){
	_address = address;
	_bus = bus;
	Wheel::interruptPin = interruptPin;


	uint8_t data = *(uint8_t*)readData(HANDSHAKE_REGISTER, 1);

	// Make sure data is correct
	if(data != _address){
		std::cerr << "Wheel handshake failed" << std::endl;
		std::cerr << "Expected: " << (int)_address << std::endl;
		std::cerr << "Got:      " << (int)data << std::endl;
		return;
	}

	// Tell user everything is ok
	std::cout << "Succesfully connected to wheel at " << (int)address << std::endl;

	// Set interrupt
	gpioSetAlertFunc(interruptPin, interrupt);
}

Wheel::~Wheel(){
	gpioSetAlertFunc(interruptPin, NULL);
}

void Wheel::setPressureAlertFunction(std::function<void()> fcn){
	_pushIntCallback = fcn;
}

void Wheel::writeData(uint8_t reg, void* data, uint8_t length){
	// Open I2C bus
	int8_t handle = i2cOpen(_bus, _address, 0);

	// Make sure connection has been established
	if(handle < 0){
		std::cerr << "Unable to connect to wheel at " << (int)_address << std::endl;
		return;
	}
	
	// Read data
	i2cWriteI2CBlockData(handle, reg, (char*)data, length);

	// Close I2C bus
	i2cClose(handle);
}

void Wheel::writeRegister(uint8_t reg){
	// Open I2C bus
	int8_t handle = i2cOpen(_bus, _address, 0);

	// Make sure connection has been established
	if(handle < 0){
		std::cerr << "Unable to connect to wheel at " << (int)_address << std::endl;
		return;
	}
	
	// Read data
	i2cWriteByte(handle, reg);

	// Close I2C bus
	i2cClose(handle);
}

void Wheel::writeRegister(uint8_t reg, uint8_t data){
	// Open I2C bus
	int8_t handle = i2cOpen(_bus, _address, 0);

	// Make sure connection has been established
	if(handle < 0){
		std::cerr << "Unable to connect to wheel at " << (int)_address << std::endl;
		return;
	}
	
	// Read data
	i2cWriteByteData(handle, reg, data);

	// Close I2C bus
	i2cClose(handle);
}
	

void* Wheel::readData(uint8_t reg, uint8_t length){
	// Open I2C bus
	int8_t handle = i2cOpen(_bus, _address, 0);

	// Make sure connection has been established
	if(handle < 0){
		std::cerr << "Unable to connect to wheel at " << (int)_address << std::endl;
		return nullptr;
	}
	
	void* data = malloc(length);
	
	// Read data
	i2cReadI2CBlockData(handle, reg, (char*)data, length);

	// Close I2C bus
	i2cClose(handle);

	return data;

}


int8_t Wheel::turnWheel(float degrees, std::function<void(int8_t)> callback){
	degrees = degrees/360;

	writeData(TURN_REGISTER, &degrees, sizeof(degrees));
	
	_turnIntCallback = callback;
	return 0;
}

int8_t Wheel::resetRotation(std::function<void(int8_t)> callback){
	writeRegister(RESET_ROTATION_REGISTER);

	_turnIntCallback = callback;
	return 0;
}



int8_t Wheel::setRotation(float degrees, std::function<void(int8_t)> callback){
	degrees = degrees/360;

	writeData(SET_ROTATION_REGISTER, &degrees, sizeof(degrees));

	_turnIntCallback = callback;
	return 0;
}


float Wheel::getRotation(){
	void* data = readData(GET_ROTATION_REGISTER, sizeof(float));
	float output = *(float*)data;
	free(data);

	return output;
}


int8_t Wheel::move(float revolutions, std::function<void(int8_t)> callback){
	writeData(MOVE_REGISTER, &revolutions, sizeof(revolutions));

	_driveIntCallback = callback;
	return 0;
}


void Wheel::drive(){
	writeRegister(DRIVE_REGISTER);
}

void Wheel::stop(){
	writeRegister(STOP_REGISTER);
}

float Wheel::getPosition(){
	void* data = readData(GET_POSITION_REGISTER, sizeof(float));
	float output = *(float*)data;
	free(data);

	return output;
}

bool Wheel::getPressureSensor(){
	void* data = readData(PRESSURE_REGISTER, sizeof(bool));
	bool output = *(bool*)data;
	free(data);

	return output; 
}


void Wheel::intHandler(int gpio, int level, uint32_t tick){
	// Get arduino status
	uint8_t status = *(uint8_t*)readData(RESPONSE_REGISTER, 1);

	if(status & STATUS_TURN_DONE){
		_turnIntCallback(1);
	}
	if(status & STATUS_DRIVE_DONE){
		_driveIntCallback(1);
	}
	if(status & STATUS_PUSH_BUTTON){
		_pushIntCallback();
	}
}
