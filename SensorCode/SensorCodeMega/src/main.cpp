#include <Arduino.h>

#include <Wire.h>
#include <XBee.h>
#include <EEPROM.h>
#include <MPU9250.h>

#define ADDRESS 0x10

#define SERVO_MAX 60

#define XBEE_SAMPLES 200

#define SERVO_COUNTER_MAX 24000 // Gives 3ms period
#define SERVO_POSITION_MAX 16000
#define SERVO_POSITION_MID 12000
#define SERVO_POSITION_MIN 800
#define TICKS_PER_DEG ((SERVO_POSITION_MAX - SERVO_POSITION_MID)/SERVO_ANGLE)

#define HANDSHAKE_REGISTER 0x01
#define SCAN_REGISTER 0x02
#define GET_ANGLE_REGISTER 0x03
#define GET_HEADING_REGISTER 0x04
#define GET_RSSI_REGISTER 0x05
#define REQUEST_REGISTER 0x06
#define GET_ROTATION_REGISTER 0x07

#define SERVO_ANGLE 60

#define INT_PIN 4

#define OFFSET_ADDRESS 0

struct RangeFinderPacket {
  float angle;
  uint16_t distance;
};

RangeFinderPacket* rangeFinderBuffer;
uint8_t scanPoints = 0;
uint16_t scanAngleLimit = 0;
uint16_t scanAngle = 0;

XBee xbee = XBee();
Rx16Response rx16 = Rx16Response();

struct XBeePacket{
  float angle;
  uint8_t rssi = 255;
};

XBeePacket xbeeData[XBEE_SAMPLES];
float heading;
uint8_t rssi;

void* response;
uint8_t responseLength = 0;
bool largeResponse = false;

int16_t servoOffset = 0;

HardwareSerial& scannerSerial = Serial2;
HardwareSerial& xbeeSerial = Serial3;

MPU9250 mpu;

struct Vector3{
  float x, y, z = 0;
};

Vector3 rotation;

void i2cRequest();
void i2cReceive(int count);
void setServo(uint16_t angle);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  xbeeSerial.begin(115200);
  scannerSerial.begin(115200);

  Serial.println("Starting...");

  xbee.setSerial(xbeeSerial);

  Wire.begin(ADDRESS);
  Wire.setClock(400000);
  Wire.onReceive(i2cReceive);
  Wire.onRequest(i2cRequest);

  pinMode(INT_PIN, OUTPUT);
  digitalWrite(INT_PIN, LOW);

  //Servo pin is D11/OC1A
  pinMode(11, OUTPUT);
  TCCR1A |= (1 << COM1A1);
  TCCR1A &= ~(1 << COM1A0) & ~(1 << WGM10) & ~(1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << CS10);
  TCCR1B &= ~(1 << WGM12) & ~(1 << CS11) & ~(1 << CS12);

  ICR1L = SERVO_COUNTER_MAX & 0xFF;
  ICR1H = SERVO_COUNTER_MAX >> 8;

  OCR1AL = SERVO_POSITION_MID & 0xFF;
  OCR1AH = SERVO_POSITION_MID >> 8;

  if(!mpu.setup(0x68)){
    Serial.println("Failed to connect to MPU");
    while(true);
  }

  Serial.println("Connected to MPU... Calibrating");

  mpu.calibrateAccelGyro();
  mpu.calibrateMag();

  Serial.println("Calibration done... Initializing Rangefinder");

  EEPROM.get(OFFSET_ADDRESS, servoOffset);

  delay(2000);
  scannerSerial.write('U');
  delay(100);
  scannerSerial.write('E');
  Serial.println("Calibrating Rangefinder...");
  //delay(15000);
  
  Serial.println("Ready!");
}

void loop() {
  static uint8_t xbeeIndex = 0;

  if(mpu.update()){
    rotation.x = mpu.getEulerX();
    rotation.y = mpu.getEulerY();
    rotation.z = mpu.getEulerZ();
  }

  xbee.readPacket();
  if(xbee.getResponse().isAvailable() && xbee.getResponse().getApiId() == RX_16_RESPONSE){
    xbee.getResponse().getRx16Response(rx16);

    for(uint8_t i = 0; i < 4; i++) *((uint8_t*)(&xbeeData[xbeeIndex].angle) + i) = rx16.getData(i);
    xbeeData[xbeeIndex].rssi = rx16.getRssi();

    uint8_t lowestIndex = 0;
    for(uint8_t i = 0; i < XBEE_SAMPLES; i++){
      if(xbeeData[i].rssi < xbeeData[lowestIndex].rssi){
        lowestIndex = i;
      }
    }

    xbeeIndex = (xbeeIndex + 1)%XBEE_SAMPLES;

    heading = xbeeData[lowestIndex].angle + 180;
    while(heading >= 360){
      heading -= 360;
    }

    rssi = xbeeData[lowestIndex].rssi;
  }
}

void serialEvent(){
  static int16_t offset = 0;
  String data = Serial.readString();

  Serial.println(data.length());

  if(data.indexOf('+') < 0 && data.indexOf('-') < 0){
    offset = data.toInt();

  }else if(data.length() == 2){
    if(data.charAt(0) == '+') offset++;
    else if(data.charAt(0) == '-') offset--;

  }else if(data.indexOf('+') >= 0){
    offset += data.substring(0, data.indexOf('+')).toInt();
    
  }else if(data.indexOf('-') >= 0){
    offset -= data.substring(0, data.indexOf('-')).toInt();
  }

  Serial.print("Offset: ");
  Serial.println((int) offset);

  EEPROM.write(OFFSET_ADDRESS, offset >> 8);
  EEPROM.write(OFFSET_ADDRESS + 1, offset & 0xFF);
}

void serialEvent2(){ // Rangefinder response
  static uint8_t index = 0;
  static uint16_t angle = 0;
  if(scanAngleLimit == 0 && scanAngle == 0) {
    while(scannerSerial.available()){
      scannerSerial.read();
    }
    return;
  }

  if(angle == 0){
    if(scanAngleLimit){
      angle = 2*SERVO_POSITION_MID - scanAngleLimit;
    }else{
      angle = scanAngle;
    }
  }

  rangeFinderBuffer[index].angle = ((float)(angle - SERVO_POSITION_MID))/TICKS_PER_DEG;
  
  uint32_t distanceBuffer;
  scannerSerial.readBytes((uint8_t*)&distanceBuffer, 4);
  rangeFinderBuffer[index].distance = (uint16_t)distanceBuffer;

  index++;

  if(index >= scanPoints){
    scanAngleLimit = 0;
    scanAngle = 0;
    index = 0;
    angle = 0;

    digitalWrite(INT_PIN, HIGH);
  }else{
    angle += 2*(scanAngleLimit - SERVO_POSITION_MID)/scanPoints;
    setServo(angle);
    scannerSerial.write('B');
  }
}

void setServo(uint16_t angle){
  angle += servoOffset;
  OCR1AL = angle & 0xFF;
  OCR1AH = angle >> 8;
}

void i2cRequest(){
  static uint8_t index = 0;
  
  if(!largeResponse){
    Wire.write((uint8_t*)response, responseLength);
    free(response);
    responseLength = 0;
    return;
  }

  Wire.write((uint8_t*)response + index, min(responseLength, 31));
  
  uint8_t padding = 0;
  if(responseLength - index < 32) padding = 31 - (responseLength - index);
  for(uint8_t i = 0; i < padding; i++) Wire.write(0);

  index += min(responseLength, 31);

  if(index >= responseLength){
    index = 0;
    largeResponse = false;
    Wire.write(31 - padding);
    free(response);
    responseLength = 0;
  }else{
    Wire.write(false);
  }
}

void i2cReceive(int count){
  uint8_t data[count];
  Wire.readBytes(data, count);

  switch(data[0]){
  case HANDSHAKE_REGISTER:
    response = malloc(sizeof(bool));
    *(uint8_t*)response = ADDRESS;
    responseLength = sizeof(bool);
    break;

  case SCAN_REGISTER:
    response = malloc(sizeof(bool));
    *(bool*)response = (bool)scanPoints;
    responseLength = sizeof(bool);
    if(scanPoints) break; // Busy

    scanPoints = data[1];
    if((unsigned int)count >= sizeof(float) + 2){
      scanAngleLimit = (uint16_t)((abs(*(float*)(data + 2)))*TICKS_PER_DEG + SERVO_POSITION_MID);
    }else{
      scanAngleLimit = SERVO_POSITION_MAX;
    }
    rangeFinderBuffer = (RangeFinderPacket*)malloc(scanPoints*sizeof(RangeFinderPacket));

    setServo(2*SERVO_POSITION_MAX - scanAngleLimit);
    scannerSerial.write('B');
    break;

  case GET_ANGLE_REGISTER:
    response = malloc(sizeof(bool));
    *(bool*)response = (bool)scanPoints;
    responseLength = sizeof(bool);
    if(scanPoints) break; // Busy

    scanPoints = 1;
    scanAngle = (uint16_t)((abs(*(float*)(data + 1)))*TICKS_PER_DEG + SERVO_POSITION_MID);
    rangeFinderBuffer = (RangeFinderPacket*)malloc(scanPoints*sizeof(RangeFinderPacket));

    setServo(scanAngle);
    scannerSerial.write('B');
    break;

  case GET_HEADING_REGISTER:
    response = malloc(sizeof(float));
    *(float*)response = heading;
    responseLength = sizeof(float);
    break;

  case GET_RSSI_REGISTER:
    response = malloc(sizeof(uint8_t));
    *(uint8_t*)response = rssi;
    responseLength = sizeof(uint8_t);
    break;

  case REQUEST_REGISTER:
    if(!responseLength) break;
    largeResponse = true;
    digitalWrite(INT_PIN, LOW);
    response = (void*)rangeFinderBuffer;
    responseLength = scanPoints*sizeof(RangeFinderPacket);
    scanPoints = 0;
    
    break;

  case GET_ROTATION_REGISTER:
    response = malloc(sizeof(rotation));
    *(Vector3*)response = rotation;
    responseLength = sizeof(rotation);
    break;
  }
}