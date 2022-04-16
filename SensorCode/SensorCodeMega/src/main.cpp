#include <Arduino.h>

#include <Wire.h>
#include <XBee.h>
#include <EEPROM.h>
#include <MPU9250.h>

#define SERVO_MAX 60

#define XBEE_SAMPLES 200

#define SERVO_COUNTER_MAX 24000U
#define SERVO_POSITION_MAX 16000U
#define SERVO_POSITION_MID 12000U
#define SERVO_POSITION_MIN 8000U
#define TICKS_PER_DEG ((SERVO_POSITION_MAX - SERVO_POSITION_MID)/SERVO_ANGLE)

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

#define ADDRESS 25

#define SERVO_ANGLE 60

#define OFFSET_ADDRESS 0

#define INT_PIN 10

#define RANGEFINDER_TIMEOUT 5000

#define TURN_TICKS_PER_REVOLUTION 16100
#define DRIVE_TICKS_PER_REVOLUTION 2600

#define STATUS_SCAN_DONE 0b0001
#define STATUS_TURN_DONE(motor) (0b001 << (motor*3 + 1))
#define STATUS_DRIVE_DONE(motor) (0b010 << (motor*3 + 1))
#define STATUS_PUSH_BUTTON(motor) (0b100 << (motor*3 + 1))

#define STATE_NONE 0
#define TURN_STATE_TURN 1
#define TURN_STATE_RESET 2
#define TURN_STATE_SET 3
#define DRIVE_STATE_MOVE 1
#define DRIVE_STATE_DRIVE 2

#define DRIVE_ERROR 25
#define TURN_ERROR 25

#define LIMIT_PIN(n) (22 + n*8)
#define PUSH_PIN(n) (23 + n*8)
#define DRIVE_MOTORA(n) (24 + n*8)
#define DRIVE_MOTORB(n) (25 + n*8)
#define TURN_MOTORA(n) (26 + n*8)
#define TURN_MOTORB(n) (27 + n*8)
#define TURN_ENCODER(n) (28 + n*8)
#define DRIVE_ENCODER(n) (29 + n*8)

#define dist(A, B, N) A - B > N - (A - B - 1) ? min(A - B, N - (A - B - 1)) : -min(A - B, N - (A - B - 1))


//#define CALIBRATION_MODE

struct RangeFinderPacket {
  float angle;
  uint16_t distance;
};

uint32_t rangeFinderTimer = 0;

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

int16_t servoOffset = 0;

HardwareSerial& scannerSerial = Serial2;
HardwareSerial& xbeeSerial = Serial3;
HardwareSerial& piSerial = Serial;

MPU9250 mpu;
MPU9250Setting setting;

struct Vector3{
  float x, y, z = 0;
};


uint8_t driveState[4] = {STATE_NONE};
uint8_t turnState[4] = {STATE_NONE};

int32_t targetTurn[4] = {0};
int32_t targetDrive[4] = {0};


int32_t turnMotorPosition[4] = {0};
int32_t driveMotorPosition[4] = {0};

uint16_t statusResponse = 0;

uint8_t lastPressureSensor[4] = {HIGH};

bool turnDirection[4];
bool driveDirection[4];

bool reverseTurnEncoder[4] = {0, 0, 0, 0};
bool reverseDriveEncoder[4] = {0, 0, 0, 0};

Vector3 rotation;
void setServo(uint16_t angle);
void print(String dat);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  xbeeSerial.begin(115200);
  scannerSerial.begin(115200);

  #ifdef CALIBRATION_MODE
  Serial.println("Starting...");
  #endif

  xbee.setSerial(xbeeSerial);

  Wire.begin();
  Wire.setClock(400000);

  // Setup wheels
  for(uint8_t i = 0; i < 4; i++){
    pinMode(DRIVE_ENCODER(i), INPUT);
    pinMode(TURN_ENCODER(i), INPUT);

    pinMode(DRIVE_MOTORA(i), OUTPUT);
    pinMode(DRIVE_MOTORB(i), OUTPUT);
    pinMode(TURN_MOTORA(i), OUTPUT);
    pinMode(TURN_MOTORB(i), OUTPUT);

    pinMode(LIMIT_PIN(i), INPUT_PULLUP);
    pinMode(PUSH_PIN(i), INPUT_PULLUP);

    digitalWrite(DRIVE_MOTORA(i), LOW);
    digitalWrite(DRIVE_MOTORB(i), LOW);
    digitalWrite(TURN_MOTORA(i), LOW);
    digitalWrite(TURN_MOTORB(i), LOW);
  }

  pinMode(INT_PIN, OUTPUT);
  digitalWrite(INT_PIN, LOW);

  //Servo pin is D11/OC1A
  pinMode(11, OUTPUT);
  TCCR1A |= (1 << COM1A1);
  TCCR1A &= ~(1 << COM1A0) & ~(1 << WGM10) & ~(1 << WGM11);
  TCCR1B |= (1 << CS10) | (1 << WGM13);
  TCCR1B &= ~(1 << WGM12) & ~(1 << CS11) & ~(1 << CS12);

  ICR1 = SERVO_COUNTER_MAX;

  #ifdef CALIBRATION_MODE
  mpu.verbose(true);
  #endif

  if(!mpu.setup(0x68)){
    #ifdef CALIBRATION_MODE
    Serial.println("Failed to connect to MPU");
    #endif
    while(true);
  }

  #ifdef CALIBRATION_MODE
  Serial.println("Connected to MPU... Calibrating");

  mpu.calibrateAccelGyro();
  mpu.calibrateMag();

  #endif
  
  #ifdef CALIBRATION_MODE
  Serial.println("Calibration done... Initializing Rangefinder");
  #endif

  servoOffset = EEPROM.read(OFFSET_ADDRESS) << 8;
  servoOffset |= EEPROM.read(OFFSET_ADDRESS + 1) & 0xFF;
  EEPROM.get(OFFSET_ADDRESS, servoOffset);

  #ifdef CALIBRATION_MODE
  Serial.print("Servo offset: ");
  Serial.println(servoOffset);
  #endif

  setServo(SERVO_POSITION_MID);

  delay(2000);
  scannerSerial.write('U');
  delay(100);
  
  #ifdef CALIBRATION_MODE
  scannerSerial.write('E');
  Serial.println("Calibrating Rangefinder...");
  delay(10000);

  Serial.println("Ready!");
  #endif
  
  
}

void loop() {
  static uint8_t xbeeIndex = 0;
  static uint8_t lastDriveEncoder[4] = {0, 0, 0, 0};
  static uint8_t lastTurnEncoder[4] = {0, 0, 0, 0};
  static uint8_t lastPressureSensor[4] = {0, 0, 0, 0};

  if(rangeFinderTimer && millis() - rangeFinderTimer > RANGEFINDER_TIMEOUT){
	scannerSerial.write('R');
	rangeFinderTimer = millis();
  }


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

    #ifdef CALIBRATION_MODE
    Serial.println("Received Data: ");
    Serial.print("\tAngle: ");
    Serial.println(xbeeData[xbeeIndex].angle);
    Serial.print("\tRSSI: -");
    Serial.println((int)xbeeData[xbeeIndex].rssi);
    #endif

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

  for(uint8_t i = 0; i < 4; i++){
    // Update motor positions
    uint8_t newDrive = digitalRead(DRIVE_ENCODER(i));
    uint8_t newTurn = digitalRead(TURN_ENCODER(i));
    if(newDrive != lastDriveEncoder[i]){
      lastDriveEncoder[i] = newDrive;

      if(newDrive == HIGH){
        if(driveDirection){
          driveMotorPosition[i]++;
        }else{
          driveMotorPosition[i]--;
        }
      }
    }

    if(newTurn != lastTurnEncoder[i]){
      lastTurnEncoder[i] = newTurn;

      if(newTurn == HIGH){
        if(turnDirection){
          turnMotorPosition[i]++;
        }else{
          turnMotorPosition[i]--;
        }
      }
    }


    // Update turn motor
    if(turnMotorPosition[i] > TURN_TICKS_PER_REVOLUTION/2){
      turnMotorPosition[i] -= TURN_TICKS_PER_REVOLUTION;
    }else if(turnMotorPosition[i] < -TURN_TICKS_PER_REVOLUTION/2){
      turnMotorPosition[i] += TURN_TICKS_PER_REVOLUTION;
    }

    if(turnState[i] == TURN_STATE_RESET){
      // Turn until the limit switch is triggered
      // Reset current and target positions when limit switch is triggered
      if(!digitalRead(LIMIT_PIN(i))){
        digitalWrite(TURN_MOTORA(i), LOW);
        digitalWrite(TURN_MOTORB(i), LOW);
        
        turnMotorPosition[i] = 0;
        targetTurn[i] = 0;

        // Trigger interrupt to tell pi the operation was completed
        statusResponse |= STATUS_TURN_DONE(i);
        digitalWrite(INT_PIN, HIGH);
        turnState[i] = STATE_NONE;
      }else{
        // Havent hit limit switch yet, keep turning
        digitalWrite(TURN_MOTORA(i), HIGH);
        digitalWrite(TURN_MOTORB(i), LOW);

        turnDirection[i] = reverseTurnEncoder[i];
      }

    }else if(dist(targetTurn[i], turnMotorPosition[i], TURN_TICKS_PER_REVOLUTION) > TURN_ERROR){
      digitalWrite(TURN_MOTORA(i), HIGH);
      digitalWrite(TURN_MOTORB(i), LOW); 
      
      turnDirection[i] = reverseTurnEncoder[i];

    }else if(dist(targetTurn[i], turnMotorPosition[i], TURN_TICKS_PER_REVOLUTION) < -TURN_ERROR){
      digitalWrite(TURN_MOTORA(i), LOW);
      digitalWrite(TURN_MOTORB(i), HIGH); 

      turnDirection[i] = !reverseTurnEncoder[i];

    }else{
      if(turnState[i] != STATE_NONE){
        statusResponse |= STATUS_TURN_DONE(i);
        digitalWrite(INT_PIN, HIGH);
        turnState[i] = STATE_NONE;
      }

      digitalWrite(TURN_MOTORA(i), LOW);
      digitalWrite(TURN_MOTORB(i), LOW);
    }

    // Check for push sensor
    if(digitalRead(PUSH_PIN(i)) && !lastPressureSensor){
      statusResponse |= STATUS_PUSH_BUTTON(i);
      lastPressureSensor[i] = HIGH;
      digitalWrite(INT_PIN, HIGH);

    }else{
      lastPressureSensor[i] = LOW;
    }

    // Update drive motor
    if(driveState[i] == DRIVE_STATE_DRIVE){
      digitalWrite(DRIVE_MOTORA(i), HIGH);
      digitalWrite(DRIVE_MOTORB(i), LOW);

      driveDirection[i] = reverseDriveEncoder[i];

    }else if(driveMotorPosition[i] < targetDrive[i] - DRIVE_ERROR){
      digitalWrite(DRIVE_MOTORA(i), HIGH);
      digitalWrite(DRIVE_MOTORB(i), LOW);

      driveDirection[i] = reverseDriveEncoder[i];

    }else if(driveMotorPosition[i] > targetDrive[i] + DRIVE_ERROR){
      digitalWrite(DRIVE_MOTORA(i), LOW);
      digitalWrite(DRIVE_MOTORB(i), HIGH);
      
      driveDirection[i] = !reverseDriveEncoder[i];

    }else{
      // If we just reached the target position, tell the Pi everything is ok
      if(driveState[i] != STATE_NONE){
        statusResponse |= STATUS_DRIVE_DONE(i);
        digitalWrite(INT_PIN, HIGH);
        driveState[i] = STATE_NONE;

      }
      
      digitalWrite(DRIVE_MOTORA(i), LOW);
      digitalWrite(DRIVE_MOTORB(i), LOW);
    }
  }
}

void serialEvent(){
#ifdef CALIBRATION_MODE
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

  servoOffset = offset;
  setServo(SERVO_POSITION_MID);

  Serial.print("Offset: ");
  Serial.println((int) offset);

  EEPROM.write(OFFSET_ADDRESS, offset >> 8);
  EEPROM.write(OFFSET_ADDRESS + 1, offset & 0xFF);

#else

  static uint8_t* data = nullptr;
  static uint8_t index = 0;
  static uint8_t length = 0;

  if(length == 0){
    length = piSerial.read();
    data = (uint8_t*)malloc(length);
    if(!piSerial.available()) return;
  }

  uint8_t addLength = piSerial.available();

  piSerial.readBytes(data + index, min(addLength, length - index));
  index += addLength;
  if(index < length) return;

  switch(data[0]){
  case HANDSHAKE_REGISTER:
    piSerial.write(ADDRESS);
    break;

  case SCAN_REGISTER:
    piSerial.write((bool)scanPoints);
    if(scanPoints) break; // Busy

    scanPoints = data[1];
    if(length >= sizeof(float) + 2){
      scanAngleLimit = (uint16_t)((abs(*(float*)(data + 2)))*TICKS_PER_DEG + SERVO_POSITION_MID);
    }else{
      scanAngleLimit = SERVO_POSITION_MAX;
    }
    rangeFinderBuffer = (RangeFinderPacket*)malloc(scanPoints*sizeof(RangeFinderPacket));

    setServo(2U*SERVO_POSITION_MAX - scanAngleLimit);
    scannerSerial.write('R');
    rangeFinderTimer = millis();
    break;

  case GET_ANGLE_REGISTER:
    piSerial.write((bool)scanPoints);
    if(scanPoints) break; // Busy

    scanPoints = 1;
    scanAngle = (uint16_t)((*(float*)(data + 1))*TICKS_PER_DEG + SERVO_POSITION_MID);
    rangeFinderBuffer = (RangeFinderPacket*)malloc(scanPoints*sizeof(RangeFinderPacket));

    setServo(scanAngle);
    scannerSerial.write('R');
    rangeFinderTimer = millis();
    break;

  case SCAN_RESPONSE_REGISTER:{
    piSerial.write(scanPoints);
    for(uint8_t i = 0; i < scanPoints; i++){
      piSerial.write((uint8_t*)&(rangeFinderBuffer[i].angle), 4);
      piSerial.write((uint8_t*)&(rangeFinderBuffer[i].distance), 2);
    }

    scanPoints = 0;
    free(rangeFinderBuffer);
    digitalWrite(INT_PIN, LOW);
    break;
  }

  case GET_HEADING_REGISTER:
    piSerial.write((uint8_t*)&heading, sizeof(heading));
    break;

  case GET_RSSI_REGISTER:
    piSerial.write(rssi);
    break;

  case GET_ROTATION_REGISTER:
    piSerial.write((uint8_t*)&rotation, sizeof(rotation));
    break;

  case GET_WHEEL_ROTATION_REGISTER:{
    float response = ((float)turnMotorPosition[data[1]])*360.0f/TURN_TICKS_PER_REVOLUTION;
    piSerial.write((uint8_t*)&response, sizeof(float));
    break;
  }

  case GET_POSITION_REGISTER:{
    float response = ((float)driveMotorPosition[data[1]])/DRIVE_TICKS_PER_REVOLUTION;
    piSerial.write((uint8_t*)&response, sizeof(float));
    break;
  }

  case RESPONSE_REGISTER:
    piSerial.write((uint8_t*)&statusResponse, sizeof(statusResponse));
    break;

  case TURN_REGISTER:{
    turnState[data[1]] = TURN_STATE_TURN;
    float angle = *(float*)(data + 2);

    targetTurn[data[1]] = turnMotorPosition[data[1]] + angle*TURN_TICKS_PER_REVOLUTION;
    break;
  }

  case RESET_ROTATION_REGISTER:
    turnState[data[1]] = TURN_STATE_RESET;
    break;

  case SET_ROTATION_REGISTER:{
    turnState[data[1]] = TURN_STATE_SET;
    float angle = *(float*)(data + 2);

    targetTurn[data[1]] = angle*TURN_TICKS_PER_REVOLUTION;
    break;
  }

  case DRIVE_REGISTER:
    driveState[data[1]] = DRIVE_STATE_DRIVE;
    break;

  case STOP_REGISTER:
    driveState[data[1]] = STATE_NONE;
    targetDrive[data[1]] = driveMotorPosition[data[1]];
    digitalWrite(DRIVE_MOTORA(data[1]), LOW);  
    digitalWrite(DRIVE_MOTORB(data[1]), LOW);  
    break;

  case PRESSURE_REGISTER:
    piSerial.write(!digitalRead(PUSH_PIN(data[1])));
    break;

  case MOVE_REGISTER:
    driveState[data[1]] = DRIVE_STATE_MOVE;

    float angle = *(float*)(data + 2);
    targetDrive[data[1]] += angle*DRIVE_TICKS_PER_REVOLUTION; 
    break;

  }

  index = 0;
  length = 0;
  free(data);

#endif
}

void serialEvent2(){ // Rangefinder response
  static uint8_t index = 0;
  static uint16_t angle = 0;

  rangeFinderTimer = 0;
  
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

  rangeFinderBuffer[index].angle = ((float)angle - SERVO_POSITION_MID)/TICKS_PER_DEG;
  rangeFinderBuffer[index].distance = scannerSerial.readString().substring(4, 9).toInt();

  index++;

  if(index >= scanPoints){
    scanAngleLimit = 0;
    scanAngle = 0;
    index = 0;
    angle = 0;
    setServo(SERVO_POSITION_MID);

    statusResponse |= STATUS_SCAN_DONE;
    digitalWrite(INT_PIN, HIGH);
  }else{
    angle += 2*(scanAngleLimit - SERVO_POSITION_MID)/scanPoints;
    setServo(angle);
    scannerSerial.write('R');
    rangeFinderTimer = millis();
  }
}

void setServo(uint16_t angle){
  angle += servoOffset;
  OCR1A = angle;
}
