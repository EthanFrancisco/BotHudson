/*
    07/18/2023
    Experimental PID Algorithm with Gyro Accelerometer (MPU-6050) for Line Follower

    WORK IN PROGRESS

    References:
    https://gist.github.com/HackMEAny/d646c6d52e708fba1c4a2354ddb5d593
*/

#include <QTRSensors.h>     // QTR Library
QTRSensors qtr;

//const uint8_t SensorCount = 8;      // No of Sensors
//uint16_t sensorValues[SensorCount];

// Bot Settings from ASICUP 2023
#define Kp 1
#define Kd 70
#define MaxSpeed 188
#define BaseSpeed 148
#define speedturn 122

// Arduino Pinouts
int PWMA = 5;
int AIN1 = 11;
int AIN2 = 12;

int PWMB = 3;
int BIN1 = 9;
int BIN2 = 8;

// Sensor Data
int lastError = 0;
int sensor[6] = {A0, A1, A2, A3, A4, A5};
int sensorValues[] = {0};
int sensorMin = 1023;
int sensorMax = 0;

void setup() {
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);

    // Add buzzer and reset pins
    while(millis() < 5000) {
        // Get data from sensors
        sensorValues[0] = analogRead(sensor[0]);
        sensorValues[1] = analogRead(sensor[1]);
        sensorValues[2] = analogRead(sensor[2]);
        sensorValues[3] = analogRead(sensor[3]);
        sensorValues[4] = analogRead(sensor[4]);
        sensorValues[5] = analogRead(sensor[5]);
        sensorValues[6] = analogRead(sensor[6]);
        sensorValues[7] = analogRead(sensor[7]);

        // Record Maximum Sensor Values
        if (sensorValues[0] > sensorMax) {
            sensorMax = sensorValues[0];
        }
        if (sensorValues[1] > sensorMax) {
            sensorMax = sensorValues[1];
        }
        if (sensorValues[2] > sensorMax) {
            sensorMax = sensorValues[2];
        }
        if (sensorValues[3] > sensorMax) {
            sensorMax = sensorValues[3];
        }
        if (sensorValues[4] > sensorMax) {
            sensorMax = sensorValues[4];
        }
        if (sensorValues[5] > sensorMax) {
            sensorMax = sensorValues[5];
        }

        // Record Minimum Sensor Values
        if (sensorValues[0] < sensorMin) {
            sensorMin = sensorValues[0];
        }
        if (sensorValues[1] < sensorMin) {
            sensorMin = sensorValues[1];
        }
        if (sensorValues[2] < sensorMin) {
            sensorMin = sensorValues[2];
        }
        if (sensorValues[3] < sensorMin) {
            sensorMin = sensorValues[3];
        }
        if (sensorValues[4] < sensorMin) {
            sensorMin = sensorValues[4];
        }
        if (sensorValues[5] < sensorMin) {
            sensorMin = sensorValues[5];
        }
    }

    // readLine equation
}

void loop() {
    // Read Sensor's Collected Values
    sensorValues[0] = analogRead(sensor[0]);
    sensorValues[1] = analogRead(sensor[1]);
    sensorValues[2] = analogRead(sensor[2]);
    sensorValues[3] = analogRead(sensor[3]);
    sensorValues[4] = analogRead(sensor[4]);
    sensorValues[5] = analogRead(sensor[5]);
    // Maps data to recieved data during calibration period, identifying line
    // Will impliment further range by following 1000 * (No. of sensor - 1) from QTR library
    sensorValues[0] = map(sensorValues[0], sensorMin, sensorMax, 0, 1000);
    sensorValues[1] = map(sensorValues[1], sensorMin, sensorMax, 0, 1000);
    sensorValues[2] = map(sensorValues[2], sensorMin, sensorMax, 0, 1000);
    sensorValues[3] = map(sensorValues[3], sensorMin, sensorMax, 0, 1000);
    sensorValues[4] = map(sensorValues[4], sensorMin, sensorMax, 0, 1000);
    sensorValues[5] = map(sensorValues[5], sensorMin, sensorMax, 0, 1000);
    // Keeps data within requested range
    // Will impliment further range by following 1000 * (No. of sensor - 1) from QTR library
    sensorValues[0] = constrain(sensorValues[0], 0, 1000);
    sensorValues[1] = constrain(sensorValues[1], 0, 1000);
    sensorValues[2] = constrain(sensorValues[2], 0, 1000);
    sensorValues[3] = constrain(sensorValues[3], 0, 1000);
    sensorValues[4] = constrain(sensorValues[4], 0, 1000);
    sensorValues[5] = constrain(sensorValues[5], 0, 1000);

    // use the documentation from pololu qtr
    // https://pololu.github.io/qtr-sensors-arduino/class_q_t_r_sensors.html#a8f2a5239ae547928284c5f81cd7ec89c

    // Equivalent of qtr.readLineBlack(sensorValues); from ASICUP 2023
    int SensorCount = sizeof(sensorValues) / sizeof(sensorValues[]);
    float SensorArraySum = 0;
    float SensorArrayAvg = 0;
    for (int i = 0; i < SensorCount; i++) {
        SensorArraySum += sensorValues[i];
    }
    SensorArrayAvg = SensorArraySum / SensorCount;

    // Turning Instructions (position = 1000 * (8 - 1) = 7000)
    uint16_t position = SensorArrayAvg
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
    int leftMotorSpeed = BaseSpeed - motorspeed;

    if (rightMotorSpeed > MaxSpeed) {
        rightMotorSpeed = MaxSpeed;
    }
    if (leftMotorSpeed > MaxSpeed) {
        leftMotorSpeed = MaxSpeed
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