#include "SensorLib.h"

Sensor* SENSOR_NAME;

Wheel* WHEEL1;
Wheel* WHEEL2;
Wheel* WHEEL3;
Wheel* WHEEL4;

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
	gpioSetAlertFunc(INT_PIN, NULL);
}

void Sensor::constructorUni(){
	// Open I2C bus
	int8_t handle = i2cOpen(_bus, _addr, 0);

	// Make sure connection has been established
	if(handle < 0){
		std::cerr << "Unable to connect to sensors." << std::endl;
		return;
	}
	
	// Read data
	uint8_t data = i2cReadByteData(handle, HANDSHAKE_REGISTER);

	// Close I2C bus
	i2cClose(handle);

	// Make sure data is correct
	if(data != _addr){
		std::cerr << "Sensor handshake failed" << std::endl;
		std::cerr << "Expected: " << (int)_addr << std::endl;
		std::cerr << "Got:      " << (int)data << std::endl;
		return;
	}


	// Tell user everything is ok
	std::cout << "Succesfully connected to sensors!" << std::endl;

	// Set interrupt
	gpioSetAlertFunc(INT_PIN, interrupt);
}


Vector3 Sensor::getRotation(){
	// Open I2C bus
	int8_t handle = i2cOpen(_bus, _addr, 0);

	// Make sure connection has been established
	if(handle < 0){
		std::cerr << "Unable to connect to sensors." << std::endl;
		return;
	}

	Vector3 output;

	i2cReadI2CBlockData(handle, GET_ROTATION_REGISTER, (char*)output, sizeof(output));

	i2cClose(handle);

	return output;
}

int8_t Sensor::getAngle(float angle, std::function<void(std::vector<RangeFinderPacket>&)> callbackFcn){
	// Open I2C bus
	int8_t handle = i2cOpen(_bus, _addr, 0);

	// Make sure connection is established
	if(handle < 0){
		std::cerr << "Unable to connect to sensor" << std::endl;
		return ERROR_CONNECTION;
	}

	char buffer[sizeof(float)];
	*(float*)buffer = angle;

	// Send command
	i2cBlockProcessCall(handle, GET_ANGLE_REGISTER, buffer, sizeof(angle));

	// Close I2C transmission
	i2cClose(handle);

	uint8_t response = *(uint8_t*)buffer;

	if(response){
		// Busy
		return ERROR_BUSY;
	}

	_callback = callbackFcn;

	return 0;
}

int8_t Sensor::scan(std::function<void(std::vector<RangeFinderPacket>&)> callbackFcn, uint8_t points, float angle){
	// Open I2C bus
	int8_t handle = i2cOpen(_bus, _addr, 0);

	// Make sure connection is established
	if(handle < 0){
		std::cerr << "Unable to connect to sensor" << std::endl;
		return ERROR_CONNECTION;
	}

	char* buffer[sizeof(points) + sizeof(angle)];
	*(uint8_t*)buffer = points;
	*(float*)(buffer + 1) = angle;

	// Send command
	i2cBlockProcessCall(handle, SCAN_REGISTER, buffer, sizeof(points) + sizeof(angle));

	// Close I2C transmission
	i2cClose(handle);

	uint8_t response = *(uint8_t*)buffer;

	if(response){
		// Busy
		return ERROR_BUSY;
	}

	_callback = callbackFcn;

	return 0;

}

void Sensor::intHandler(int gpio, int level, uint32_t tick){
	// Return if not rising edge
	if(level != 1) return;

	void* buffer;
	uint8_t bufferSize = 0;

	int8_t handle = i2cOpen(_bus, _addr, 0);

	if(handle < 0){
		std::cerr << "Unable to connect to sensor" << std::endl;
		return;
	}

	uint8_t done = false;
	do{
		char data[32];
		i2cReadI2CBlockData(handle, REQUEST_REGISTER, data, 32);

		done = data[31];

		if(done){
			bufferSize += done;
			buffer = realloc(buffer, bufferSize);
			memcpy(buffer + bufferSize - done, (void*)data, done);

		}else{
			bufferSize += 31;
			buffer = realloc(buffer, bufferSize);
			memcpy(buffer + bufferSize - 31, (void*)data, 31);
		}
	}while(!done);

	i2cClose(handle);

	std::vector<RangeFinderPacket> output(bufferSize/sizeof(RangeFinderPacket));
	for(uint8_t i = 0; i < output.size(); i++){
		output[i] = ((RangeFinderPacket*)buffer)[i];
	}

	_callback(output);

}

float Sensor::getHeading(){
	float output;
	int8_t handle = i2cOpen(_bus, _addr, 0);

	if(handle < 0){
		std::cerr << "Unable to connect to sensor" << std::endl;
		return 0;
	}

	i2cReadI2CBlockData(handle, GET_HEADING_REGISTER, (char*)(&output), sizeof(float));

	i2cClose(handle);

	return output;
}

uint8_t Sensor::getRSSI(){
	uint8_t output;
	int8_t handle = i2cOpen(_bus, _addr, 0);

	if(handle < 0){
		std::cerr << "Unable to connect to sensor" << std::endl;
		return 255;
	}

	i2cReadI2CBlockData(handle, GET_RSSI_REGISTER, (char*)(&output), 1);

	i2cClose(handle);

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

