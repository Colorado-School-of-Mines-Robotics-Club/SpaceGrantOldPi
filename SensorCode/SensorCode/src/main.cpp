#include <Arduino.h>

#include <Wire.h>
#include <XBee.h>
#include <AltSoftSerial.h>
#include <SoftwareSerial.h>

#define I2C_ADDRESS 0x04

#define RADAR_DATA_POINTS 8
#define RADAR_ANGLE 90
#define BUFFER_SIZE 1

#define XBEE_SAMPLES 100

// Servo pin 11 (OC2A)
// Frequency: 244.14 Hz (128 prescalar w/ phase correct clocking)
// Duty cycle for 1.5ms: 36.621%
// Duty cycle for 1ms: 24.414%
// Duty cycle for 2ms: 48.828%

AltSoftSerial xbeeSerial; // RX: 8, TX: 9
SoftwareSerial rangefinderSerial(2, 3); // RX: 2, TX: 3


struct {
  float angle; // Degrees
  uint32_t distance; // mm
  uint32_t timestamp = 0; // millisecs since start of program
} rangefinderBuffer[RADAR_DATA_POINTS*BUFFER_SIZE];
uint8_t packetPointer = 0;
uint8_t index = 0;

float servoAngle = 0;
bool servoDirection = true;

// XBee global vars
XBee xbee = XBee();
Rx16Response rx16 = Rx16Response();

struct XBeeData{
  union{
    float f;
    uint8_t b[4];
  }angle;
  uint8_t rssi = 255;
};
XBeeData xbeeData[XBEE_SAMPLES];
uint16_t heading = 0;
uint16_t xbeePointer = 0;

uint8_t registerPointer = 0;

void i2cRequest(){
  if(registerPointer == 0x01){ //Read rangefinder
    for(uint8_t i = index; i < index + 3 && i < RADAR_DATA_POINTS*BUFFER_SIZE; i++){
      for(uint8_t j = 0; j < 4; j++){
        Wire.write(((byte*)(&(rangefinderBuffer[i].angle)))[j]);
      }
      for(uint8_t j = 0; j < 2; j++){
        Wire.write(((byte*)(&(rangefinderBuffer[i].distance)))[j]);
      }
      for(uint8_t j = 0; j < 4; j++){
        Wire.write(((byte*)(&(rangefinderBuffer[i].timestamp)))[j]);
      }
      rangefinderBuffer[i].timestamp = 0;
    }
    
    index += 3;
    if(index < RADAR_DATA_POINTS*BUFFER_SIZE){
      Wire.write(true);
    }else{
      Wire.write(false);
      registerPointer = 0;
      index = 0;
    }
  }
}

void i2cReceive(int count){
  // Read register
  registerPointer = Wire.read();
}



// Clamps input value between min and max
float clamp(float minimum, float maximum, float in){
  if(in < minimum){
    return minimum;
  }else if(in > maximum){
    return maximum;
  }
  return in;
}

void setServo(float angle){
  OCR2A = (uint8_t)((clamp(-90, 90, angle)/180 + 0.5)*(48.828*255/100 - 24.414*255/100) + 24.414*255/100);
}

void setup() {
  //Begin Serial
  Serial.begin(9600); // USB Serial
  xbeeSerial.begin(9600);
  rangefinderSerial.begin(115200);
  
  xbee.setSerial(xbeeSerial);

  // Begin I2C
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(i2cReceive);
  Wire.onRequest(i2cRequest);

  //pinMode(11, OUTPUT);
  DDRB |= 1 << 3;
  TCCR2A = B10000001;
  TCCR2B = B00000101;

  delay(1000); // Wait for rangefinder to boot
  rangefinderSerial.write('U'); // Write init bit to rangefinder
  delay(100);
  rangefinderSerial.write('R');
  //rangefinderSerial.write('L');

  Serial.println("Starting...");

}

void loop() {
  if(packetPointer >= RADAR_DATA_POINTS*BUFFER_SIZE){
    packetPointer = 0;
  }

  // If the rangefinder has returned another datapoint add it to the buffer
  if(rangefinderSerial.available()){
    if(registerPointer != 0x01){
      // Add angle and timestamp
      rangefinderBuffer[packetPointer].angle = servoAngle;
      rangefinderBuffer[packetPointer].distance = 0;
      rangefinderBuffer[packetPointer].timestamp = millis();
  
      // If there are multiple values in the serial buffer add the most recent.
      //while(rangefinderSerial.available()){
      rangefinderBuffer[packetPointer].distance = rangefinderSerial.readString().substring(4, 9).toInt();
        /*if(rangefinderSerial.available() >= 4){
          uint32_t data;
          //for(uint8_t i = 0; i < 4; i++) ((uint8_t*)(&data))[3 - i] = rangefinderSerial.read();
          //rangefinderBuffer[packetPointer].distance = data;
        }else{
          // Just in case there are less than 4 bytes in the buffer for some reason, clear the buffer
          rangefinderSerial.read();
        }*/
      //}
      while(rangefinderSerial.available()){
        rangefinderSerial.read();
      }
      Serial.print("Angle: ");
      Serial.print(rangefinderBuffer[packetPointer].angle);
      Serial.print(" Distance: ");
      Serial.print(rangefinderBuffer[packetPointer].distance);
      Serial.print(" Time: ");
      Serial.println(rangefinderBuffer[packetPointer].timestamp);
      
      packetPointer++;
  
      // Calculate and set new servo angle
      if(abs(servoAngle) > RADAR_ANGLE - RADAR_ANGLE*2/RADAR_DATA_POINTS) servoDirection = !servoDirection;
      servoAngle += (servoDirection*2 - 1)*RADAR_ANGLE*2/RADAR_DATA_POINTS;
      //servoAngle = RADAR_ANGLE*sin(millis()/1000.0f * RADAR_FREQUENCY);
      setServo(servoAngle);
    }else{ //In read operation, pause measurements
      while(rangefinderSerial.available()){
        Serial.read();
      }
    }

    while(rangefinderSerial.available()){
      Serial.read();
    }
    rangefinderSerial.write('R');
  }



  
  xbee.readPacket();
  if(xbee.getResponse().isAvailable() && xbee.getResponse().getApiId() == RX_16_RESPONSE){
    xbee.getResponse().getRx16Response(rx16);

    for(uint8_t i = 0; i < 4; i++) xbeeData[xbeePointer].angle.b[3 - i] = rx16.getData(i);
    xbeeData[xbeePointer].rssi = rx16.getRssi();

    uint16_t lowestIndex = 0;
    for(uint16_t i = 0; i < XBEE_SAMPLES; i++){
      if(xbeeData[i].rssi < xbeeData[lowestIndex].rssi){
        lowestIndex = i;
      }
    }

    xbeePointer++;
    if(xbeePointer >= XBEE_SAMPLES){
      xbeePointer = 0;
    }

    Serial.println("Received");

    heading = xbeeData[lowestIndex].angle.f;
    
  }
}