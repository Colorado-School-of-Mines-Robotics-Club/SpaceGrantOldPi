#include "SensorLib.h"

Sensor* SENSOR_NAME;

Wheel* WHEEL1;
Wheel* WHEEL2;
Wheel* WHEEL3;
Wheel* WHEEL4;

//char SERTTY[] = "/dev/ttyS0";
char SERTTY[] = "/dev/serial0";

Sensor::Sensor(uint8_t address, uint8_t bus)/* : arduino("/dev/ttyACM0")*/{
	_addr = address;
	_bus = bus;
	constructorUni();
}

Sensor::Sensor()/* : arduino("/dev/ttyACM0")*/{
	_addr = 0x19;
	_bus = 1;
	constructorUni();
}

// Deconstructor removes the interrput to avoid nullptr errors
Sensor::~Sensor(){
	close(arduinoPort);
	gpioSetAlertFunc(SENSOR_INT_PIN, NULL);
}

uint8_t Sensor::readRegister(uint8_t reg){
	tcflush(arduinoPort, TCIOFLUSH);
	uint8_t data[2] = {1, reg};
	write(arduinoPort, data, 2);
	read(arduinoPort, data, 1);
	return data[0];
}

void Sensor::readRegister(uint8_t reg, char* buffer, uint8_t length){
	tcflush(arduinoPort, TCIOFLUSH);
	uint8_t txBuffer[2] = {1, reg};
	write(arduinoPort, txBuffer, 2);
	read(arduinoPort, buffer, length);
}

uint8_t Sensor::writeRegister(uint8_t reg, char* data, uint8_t length){
	tcflush(arduinoPort, TCIOFLUSH);
	uint8_t txBuffer[length + 2];
	txBuffer[0] = length + 1;
	txBuffer[1] = reg;
	memcpy(txBuffer + 2, data, length);

	write(arduinoPort, txBuffer, length + 2);

	uint8_t output;
	read(arduinoPort, &output, 1);
	
	return output;
}

void Sensor::constructorUni(){
	arduinoPort = open("/dev/ttyACM0", O_RDWR);

	if (arduinoPort < 0) {
    	printf("Error %i from open: %s\n", errno, strerror(errno));
	}

	struct termios tty;
	if(tcgetattr(arduinoPort, &tty) != 0) {
		printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	}

	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~CRTSCTS;
	tty.c_cflag |= CREAD | CLOCAL;
	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure	
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG;
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	tty.c_cc[VTIME] = 50;
	tty.c_cc[VMIN] = 1;
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	if (tcsetattr(arduinoPort, TCSANOW, &tty) != 0) {
		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
	}
	sleep(5);

	uint8_t data = readRegister(HANDSHAKE_REGISTER);

	// Make sure data is correct
	if(data != _addr){
		std::cerr << "Sensor handshake failed" << std::endl;
		std::cerr << "Expected: " << (int)_addr << std::endl;
		std::cerr << "Got:      " << (int)data << std::endl;
		return;
	}
	


	// Tell user everything is ok
	std::cout << "Succesfully connected to sensors!" << std::endl;

	gpioSetAlertFunc(SENSOR_INT_PIN, interrupt);
}


Vector3 Sensor::getRotation(){
	Vector3 output;
	readRegister(SENSOR_GET_ROTATION_REGISTER, (char*)&output, sizeof(output));

	if(    output.x == 0   ||     output.y == 0   ||     output.z == 0 ||
	   abs(output.x) > 180 || abs(output.y) > 180 || abs(output.z) > 360){ // Bad data

			return getRotation();
	   }

	return output;
}

int8_t Sensor::getAngle(float angle, std::function<void(std::vector<RangeFinderPacket>&)> callbackFcn){

	uint8_t response = writeRegister(GET_ANGLE_REGISTER, (char*)&angle, sizeof(angle));

	if(response){
		// Busy
		return ERROR_BUSY;
	}

	_callback = callbackFcn;

	return 0;
}

int8_t Sensor::scan(std::function<void(std::vector<RangeFinderPacket>&)> callbackFcn, uint8_t points, float angle){
	angle = std::abs(angle);
	uint8_t response = getAngle(-angle, callbackFcn);

	if(response){
		return ERROR_BUSY;
	}

	scanAngle = angle;
	dataRequired = points - 1;

	return response;

	/*char buffer[sizeof(points) + sizeof(angle)];
	*(uint8_t*)buffer = points;
	*(float*)(buffer + sizeof(points)) = angle; 

	uint8_t response = writeRegister(SCAN_REGISTER, buffer, sizeof(points) + sizeof(angle));

	if(response){
		// Busy
		return ERROR_BUSY;
	}

	_callback = callbackFcn;
	return 0;*/

}

void Sensor::intHandler(int pin, int level, uint32_t tick){
	static int totalReceived = 0;
	responseData.emplace_back();
	totalReceived++;

	tcflush(arduinoPort, TCIOFLUSH);
	uint8_t txBuffer[2] = {1, REQUEST_REGISTER};
	write(arduinoPort, txBuffer, 2);

	uint8_t length;
	read(arduinoPort, &length, 1);

	read(arduinoPort, (uint8_t*)&(responseData.back().angle), 4);
	read(arduinoPort, (uint8_t*)&(responseData.back().distance), 2);

	if(responseData.back().distance > 10000 || abs(responseData.back().angle) > 180){ // Bad data
		totalReceived--;
		responseData.pop_back();
	}

	if(totalReceived > dataRequired){
		totalReceived = 0;
		dataRequired = 0;
		scanAngle = 0;
		dataRequired = 0;

		_callback(responseData);
		responseData.clear();

		return;
	}

	getAngle(2*scanAngle*totalReceived/dataRequired - scanAngle, _callback);
/*tcflush(arduinoPort, TCIOFLUSH);
	uint8_t txBuffer[2] = {1, REQUEST_REGISTER};
	write(arduinoPort, txBuffer, 2);

	std::vector<RangeFinderPacket> data;

	while(true){
		uint8_t packetInfo[2];
		read(arduinoPort, packetInfo, 2);

		int received = 0;
		uint8_t checksum = packetInfo[0];

		data.resize(packetInfo[0]);
		for(uint8_t i = 0; i < packetInfo[0]; i++){
			received += read(arduinoPort, (uint8_t*)&(data[i].angle), 4);
			received += read(arduinoPort, (uint8_t*)&(data[i].distance), 2);

			for(uint8_t j = 0; j < sizeof(float); j++){
				checksum += *(((uint8_t*)&(data[i].angle)) + j);
			}
			for(uint8_t j = 0; j < sizeof(uint16_t); j++){
				checksum += *(((uint8_t*)&(data[i].distance)) + j);
			}
		}

		std::cout << "Received: " << received << std::endl;

		uint8_t checksumResponse = checksum != packetInfo[1];
		write(arduinoPort, &checksumResponse, 1);

		if(!checksumResponse){
			std::cout << "Checksum passed" << std::endl;
			break;
		}

		std::cout << "Checksum failed" << std::endl;
	}

	_callback(data);*/
}


float Sensor::getHeading(){
	float output;
	readRegister(GET_HEADING_REGISTER, (char*)&output, sizeof(output));

	return output;
}

uint8_t Sensor::getRSSI(){
	return readRegister(GET_RSSI_REGISTER);
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