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
	int8_t handle;
	uint8_t data;
	/*int8_t handle = i2cOpen(_bus, _addr, 0);

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
	}*/

	handle = i2cOpen(_bus, GYRO_ADDRESS, 0);

	// Make sure connection has been established
	if(handle < 0){
		std::cerr << "Unable to connect to sensors." << std::endl;
		return;
	}

	data = i2cReadByteData(handle, WHOAMI_REGISTER);


	if(data != 0x71){
		std::cerr << "Gyro handshake failed" << std::endl;
		std::cerr << "Expected: " << 0x71 << std::endl;
		std::cerr << "Got:      " << (int)data << std::endl;
		return;
	}

	// Tell user everything is ok
	std::cout << "Succesfully connected to sensors!" << std::endl;

	// Initialize gyro
	i2cWriteByteData(handle, CONFIG_REGISTER, 0);
	i2cWriteByteData(handle, GYRO_CONFIG_REGISTER, 0b11);
	i2cWriteByteData(handle, ACCEL_CONFIG_REGISTER, 0);
	i2cWriteByteData(handle, ACCEL_CONFIG2_REGISTER, 0b1000);
	i2cWriteByteData(handle, FIFO_EN_REGISTER, 0b01111000);
	i2cWriteByteData(handle, GYRO_INT_CFG_REGISTER, 0b00010000);
	i2cWriteByteData(handle, GYRO_INT_EN_REGISTER, 0b00000001);
	i2cWriteByteData(handle, USER_CTL_REGISTER, 0b01000100);
	//i2cWriteByteData(handle, MAGNETOMETER_CTL_REGISTER, 0b00010010);

	std::cout << "Gyro initialized" << std::endl;

	i2cClose(handle);

	// Set interrupt
	gpioSetAlertFunc(INT_PIN, interrupt);
	gpioSetAlertFunc(GYRO_INT_PIN, interrupt);

	lastTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

 
void Sensor::gyroIntHandler(int gpio, int level, uint32_t tick){
	if(!level) return;
	
	int8_t handle = i2cOpen(_bus, GYRO_ADDRESS, 0);

	// Make sure connection has been established
	if(handle < 0){
		std::cerr << "Unable to connect to sensors." << std::endl;
		return;
	}

	// Check if fifo is ready
	uint16_t count = ((i2cReadByteData(handle, FIFO_COUNTH)  & 0b11111) << 8) & i2cReadByteData(handle, FIFO_COUNTL);
	if(count < 12) {
		i2cClose(handle);
		return;
	}

	uint32_t currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	uint32_t deltaTime = currentTime - lastTime;
	lastTime = currentTime;

	
	int16_t gyro[3] = {0, 0, 0};
	int16_t accel[3] = {0, 0, 0};

	char fifo[12];
	i2cReadI2CBlockData(handle, FIFO_REGISTER, fifo, 12);

	memcpy(accel, fifo, 6);
	memcpy(gyro, fifo + 6, 6);


	_accelEstimate.x = (_accelEstimate.x + accel[0]*9.81/2*ACCEL_WEIGHT)/(ACCEL_WEIGHT + 1);
	_accelEstimate.y = (_accelEstimate.y + accel[1]*9.81/2*ACCEL_WEIGHT)/(ACCEL_WEIGHT + 1);
	_accelEstimate.z = (_accelEstimate.z + accel[2]*9.81/2*ACCEL_WEIGHT)/(ACCEL_WEIGHT + 1);

	Vector3 accelRotation;

	accelRotation.x = atan2(_accelEstimate.y, _accelEstimate.z);
	accelRotation.y = atan2(-_accelEstimate.x, _accelEstimate.z);

	_rotation.x = ((_rotation.x + gyro[0]*deltaTime/250.0f*M_PI/180)*GYRO_WEIGHT + accelRotation.x)/(GYRO_WEIGHT + 1)*180/M_PI;
	_rotation.y = ((_rotation.y + gyro[1]*deltaTime/250.0f*M_PI/180)*GYRO_WEIGHT + accelRotation.y)/(GYRO_WEIGHT + 1)*180/M_PI;
	_rotation.z += gyro[2]*deltaTime/250.0f*180/M_PI;

	i2cClose(handle);

	std::cout << "GyroX: " << (int)gyro[0]<< "\tGyroY: " << (int)gyro[1] << "\tGyroZ: " << (int)gyro[2] << std::endl;
	std::cout << "AccelX: " << (int)accel[0]<< "\tAccelY: " << (int)accel[1] << "\tAccelZ: " << (int)accel[2] << std::endl;
	

}

Vector3 Sensor::getRotation(){
	return _rotation;
}

int8_t Sensor::getAngle(float angle, std::function<void(RangeFinderPacket&)> callbackFcn){
	// Make sure the arduino is free
	if(_interruptState != INT_STATE_NONE){
		return ERROR_BUSY;
	}

	// Open I2C bus
	int8_t handle = i2cOpen(_bus, _addr, 0);

	// Make sure connection is established
	if(handle < 0){
		std::cerr << "Unable to connect to sensor" << std::endl;
		return ERROR_CONNECTION;
	}

	// Send command
	i2cWriteI2CBlockData(handle, ANGLE_REGISTER, (char*)(&angle), sizeof(angle));

	// Close I2C transmission
	i2cClose(handle);

	// Set interrupt state
	_interruptState = INT_STATE_GET_ANGLE;

	_angleCallback = callbackFcn;

	return 0;
}

int8_t Sensor::scan(std::function<void(std::vector<RangeFinderPacket>&)> callbackFcn){
	// Make sure the arduino is free
	if(_interruptState != INT_STATE_NONE){
		return ERROR_BUSY;
	}
	
	// Open I2C bus
	int8_t handle = i2cOpen(_bus, _addr, 0);

	// Make sure connection is established
	if(handle < 0){
		std::cerr << "Unable to connect to sensor" << std::endl;
		return ERROR_CONNECTION;
	}

	// Send command
	i2cWriteByte(handle, SCAN_REGISTER);

	// Close I2C transmission
	i2cClose(handle);

	_interruptState = INT_STATE_SCAN;

	_scanCallback = callbackFcn;

	return 0;

}

void Sensor::intHandler(int gpio, int level, uint32_t tick){
	// Return if not rising edge
	if(level != 1) return;

	switch(_interruptState){
	case INT_STATE_GET_ANGLE:{
		RangeFinderPacket data;
		int8_t handle = i2cOpen(_bus, _addr, 0);

		if(handle < 0){
			std::cerr << "Unable to connect to sensor" << std::endl;
			return;
		}

		// Read data into data variable
		i2cReadI2CBlockData(handle, READ_ANGLE_REGISTER, (char*)(&data), 6);


		i2cClose(handle);

		_interruptState = INT_STATE_NONE;

		_angleCallback(data);

		break;
	}
	case INT_STATE_SCAN:{
		std::vector<RangeFinderPacket> scanData;

		int8_t handle = i2cOpen(_bus, _addr, 0);

		if(handle < 0){
			std::cerr << "Unable to connect to sensor" << std::endl;
			return;
		}

		for(uint8_t i = 0; i < (uint8_t)ceil(SCAN_POINTS*6/32.0f); i++){
			uint8_t bufferSize = 32/6;
			RangeFinderPacket data[bufferSize];

			i2cReadI2CBlockData(handle, READ_SCAN_REGISTER, (char*)(&data), bufferSize*6);

			for(int j = 0; j < bufferSize; j++){
				if(scanData.size() >= SCAN_POINTS) break;

				scanData.push_back(data[j]);
			}
		}	

		i2cClose(handle);

		_interruptState = INT_STATE_NONE;

		_scanCallback(scanData);

		break;
	}

	}

}

float Sensor::getHeading(){
	float output;
	int8_t handle = i2cOpen(_bus, _addr, 0);

	if(handle < 0){
		std::cerr << "Unable to connect to sensor" << std::endl;
		return 0;
	}

	i2cReadI2CBlockData(handle, HEADING_REGISTER, (char*)(&output), sizeof(float));

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

	i2cReadI2CBlockData(handle, RSSI_REGISTER, (char*)(&output), 1);

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
	
	char* data = new char[length];
	
	// Read data
	i2cReadI2CBlockData(handle, reg, data, length);

	// Close I2C bus
	i2cClose(handle);

	return (void*)data;

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
	return *(float*)readData(GET_ROTATION_REGISTER, sizeof(float));
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
	return *(float*)readData(GET_POSITION_REGISTER, sizeof(float));
}

bool Wheel::getPressureSensor(){
	return *(bool*)readData(PRESSURE_REGISTER, sizeof(bool));
}


void Wheel::intHandler(int gpio, int level, uint32_t tick){
	// Get arduino status
	uint8_t status = *(uint8_t*)readData(RESPONSE_REGISTER, 1);

	if(status & STATUS_TURN_DONE){
		_turnIntCallback(1);
		std::cout << "Status turn done" << std::endl;
	}
	if(status & STATUS_DRIVE_DONE){
		std::cout << "Status drive done" << std::endl;
		_driveIntCallback(1);
	}
	if(status & STATUS_PUSH_BUTTON){
		std::cout << "Status push pressed" << std::endl;
		_pushIntCallback();
	}
}

