/*

  Francisco, Ethan Philip T.
  04/20/2023
  BotHudson Sumobot

*/

// Remove all serials syntax before competition

#include <QTRSensors.h>
#include <SharpIR.h>

QTRSensors qtr;
SharpIR sensor(SharpIR::GP2Y0A21YK0F, A0);

const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];

// Arena Settings
#define BLK 700             // Arena Color Value - ??? (Higher)
#define WHT 80              // Border Color Value - ??? (Lower)
#define QTR_THRESHOLD 750  // microseconds (need tuning per each environment)

// Speed Settings
#define speedTurn 80       // Default - 80
#define speedForward 150   // Default - 255
#define speedBackward 150  // Default - 255
#define speedCharge 255    // Default - 255

// Left Motor Pins
#define PWMA 5   // speedControl
#define AIN1 11  // pinDirection
#define AIN2 12  // pinDirection

// Right Motor Pins
#define PWMB 3  // speedControl
#define BIN1 9  // pinDirection
#define BIN2 8  // pinDirection

// Sensor Pin Configuration
#define IROutput A0  // SharpIR
//#define USTrig 12    // Ultrasonic Trig
//#define USEcho A3    // Ultrasonic Echo

void setup() {
  Serial.begin(9600);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ A1, A2, A3, A4 }, SensorCount);
  qtr.setEmitterPin(2);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  delay(3000);
}

void loop() {

  // Sharp IR (cm output)
  int distance = sensor.getDistance();
  //Serial.println(distance);

  // QTR Sensor
  // High Value (2500) - Black
  // Low Value (0) - White
  qtr.read(sensorValues);
  //Serial.println(sensorValues);

  // Sumobot Algorithm and Conditions
  if (sensorValues[0] < QTR_THRESHOLD) {
    // Leftmost Sensor Detected the Border
    move(1, speedBackward, 0);
    move(0, speedBackward, 0);
    delay(250);
    move(1, speedTurn, 1);
    move(0, speedTurn, 0);
    delay(250);
    move(1, speedForward, 1);
    move(0, speedForward, 1);
  } else if (sensorValues[3] < QTR_THRESHOLD) {
    // Rightmost Sensor Detected The Border
    move(1, speedBackward, 0);
    move(0, speedBackward, 0);
    delay(250);
    move(1, speedTurn, 0);
    move(0, speedTurn, 1);
    delay(250);
    move(1, speedForward, 1);
    move(0, speedForward, 1);
  } else {
    if (distance < 30) {
      // SharpIR Detection
      move(1, speedCharge, 1);
      move(0, speedCharge, 1);
    } else {
// Idle Search
idleSearch:
      move(1, speedForward, 1);
      move(0, speedForward, 1);
      delay(500);
      move(1, speedTurn, 1);
      move(0, speedTurn, 0);
      delay(50);
      move(1, speedForward, 1);
      move(0, speedForward, 1);
      delay(500);
      move(1, speedTurn, 0);
      move(0, speedTurn, 1);
      delay(50);
      goto idleSearch;
    }
  }
}

// move() function declaration
void move(int motor, int speed, int direction) {
  boolean inPin1 = HIGH, inPin2 = LOW;
  if (direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }
  if (direction == 0) {
    inPin1 = LOW;
    inPin2 = HIGH;
  }
  if (motor == 0) {
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }
  if (motor == 1) {
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}