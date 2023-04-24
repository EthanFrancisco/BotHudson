/*

  Francisco, Ethan Philip T.
  04/17/2023
  PID Algorithm for Line Follower Mobot

*/

#include <QTRSensors.h> // QTR library
QTRSensors qtr;

const uint8_t SensorCount = 5; // No of sensors
uint16_t sensorValues[SensorCount];

#define Kp 1          // Proportional Default - 2
#define Kd 70         // Derivative Default - 40
#define MaxSpeed 188  // Default 255
#define BaseSpeed 148 // Default - 155
#define speedturn 112  // Default - 100

/*

Optimal presets for ASICUP competition

ethan's
Kp = 2
Kd = 40
mS = 210
bS = 170
st = 110

ethan's 2nd
Kp = 1
Kd = 70
mS = 188
bS = 148
st = 112

louis'
Kp = 1
Kd = 62
mS = 218
bS = 168
st = 112

deretso sa box (experimental)
Kp = 1
Kd = 90
mS = 235
bS = 155
st = 120

*/

// Left Motor Pins
int PWMA = 5;   // speedControl
int AIN1 = 11;  // pinDirection
int AIN2 = 12;  // pinDirection

// Right Motor Pins
int PWMB = 3;   // speedControl
int BIN1 = 9;   // pinDirection
int BIN2 = 8;   // pinDirection

int lastError = 0;

void setup() {
  qtr.setTypeRC(); // Sensor Type (Analog or RC)
  qtr.setSensorPins((const uint8_t[]) {
    A0, A1, A2, A3, A4
  }, SensorCount);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1500); // Initial Delay

  // Sensor Calibration
  int i;
  for (int i = 0; i < 85; i++) {
    if (i < 25 || i > 75) {
      move(0, 80, 1);
      move(1, 80, 0);
    } else {
      move(0, 80, 0);
      move(1, 80, 1);
    }
    qtr.calibrate();
    delay(20);
  }
  delay(500); // Post-Calibration Delay
}

void loop() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  if (position > 6700) {
    move(1, speedturn, 1);
    move(0, speedturn, 0);
    return;
  }
  if (position < 300) {
    move(1, speedturn, 0);
    move(0, speedturn, 1);
    return;
  }

  // PID Algorithm
  int error = position - 3500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  int rightMotorSpeed = BaseSpeed + motorSpeed;
  int leftMotorSpeed = BaseSpeed - motorSpeed;

  if (rightMotorSpeed > MaxSpeed) {
    rightMotorSpeed = MaxSpeed;
  }
  if (leftMotorSpeed > MaxSpeed) {
    leftMotorSpeed = MaxSpeed;
  }
  if (rightMotorSpeed < 0) {
    rightMotorSpeed = 0;
  }
  if (leftMotorSpeed < 0) {
    leftMotorSpeed = 0;
  }

  move(1, rightMotorSpeed, 1);
  move(0, leftMotorSpeed, 1);
}

// move() func declaration
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
