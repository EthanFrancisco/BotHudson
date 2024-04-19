/*
	18/04/2023
	PID Algorithm with Speed Control for Line Follower Mobot
*/

#include <QTRSensors.h>	// QTR library
QTRSensors qtr;

const uint8_t SensorCount = 8;	// No. of sensors
uint16_t sensorValues[SensorCount];

#define Kp 0.2			// Proportional Default - 0.2
#define Kd 4			// Derivative Default - 4
#define Ksr 0			// Speed Reduction Default - 0
#define MaxSpeed 255	// Default - 255
#define BaseSpeed 255	// Default - 255
#define speedturn 255	// Default - 255

#define userButton 0	// User Button Pin

// Left Motor Pins
int PWMA = 6;	// speedControl
int AIN1 = 5;	// pinDirection
int AIN2 = 4;	// pinDirection

// Right Motor Pins
int PWMB = 3;	// speedControl
int BIN1 = 2;	// pinDirection
int BIN2 = 1;	// pinDirection

int lastError = 0;

void setup() {
	qtr.setTypeRC();	// Sensor Type (Analog or RC)
	qtr.setSensorPins((const uint8_t[]) {
		19, 18, 17, 16, 15, 14, 8, 7
	}, SensorCount);
  	pinMode(PWMA, OUTPUT);
  	pinMode(AIN1, OUTPUT);
  	pinMode(AIN2, OUTPUT);
  	pinMode(PWMB, OUTPUT);
  	pinMode(BIN1, OUTPUT);
  	pinMode(BIN2, OUTPUT);
	pinMode(userButton, INPUT_PULLUP);
  	delay(200);	// Initial Delay Default - 1500

	// Sensor Calibration
	int i;
	for(int i = 0; i < 100; i++) {
	if(i < 25 || i > 75) {
    	move(0, 60, 1);
    	move(1, 60, 0);
	} else {
    	move(0, 60, 0);
    	move(1, 60, 1);
	}
	qtr.calibrate();
	delay(10);	// Default - 20
	}
	delay(100);	// Post-Calibration Delay Default - 500
	while(digitalRead(userButton) == HIGH) {
		move(0, 0, 0);
		move(1, 0, 0);
	}
	delay(1000);
}

void loop() {
	uint16_t position = qtr.readLineBlack(sensorValues);
	if(position > 6700) {
		move(1, speedturn, 0);
		move(0, speedturn, 1);
		return;
	}
	if(position < 300) {
		move(1, speedturn, 1);
		move(0, speedturn, 0);
		return;
	}

	// PID Algorithm
	int error = position - 3500;
	int motorSpeed = Kp * error + Kd * (error - lastError);
	lastError = error;

	double speedError = Ksr * (exp(1) * ((8 * (8 + 1)) / 2));

	int rightMotorSpeed = BaseSpeed + motorSpeed - speedError;
	int leftMotorSpeed = BaseSpeed - motorSpeed - speedError;

	if(rightMotorSpeed > MaxSpeed) {
		rightMotorSpeed = MaxSpeed;
	}
	if(leftMotorSpeed > MaxSpeed) {
		leftMotorSpeed = MaxSpeed;
	}
	if(rightMotorSpeed < MaxSpeed) {
		rightMotorSpeed = rightMotorSpeed;
	}
	if(leftMotorSpeed < MaxSpeed) {
		leftMotorSpeed = leftMotorSpeed;
	}
	if(rightMotorSpeed < 0) {
		rightMotorSpeed = 0;
	}
	if(leftMotorSpeed < 0) {
		leftMotorSpeed = 0;
	}

	move(1, rightMotorSpeed, 1);
	move(0, leftMotorSpeed, 1);
}

// move() func declaration
void move(int motor, int speed, int direction) {
	boolean inPin1 = HIGH, inPin2 = LOW;
	if(direction == 1) {
		inPin1 = HIGH;
		inPin2 = LOW;
	}
	if(direction == 0) {
		inPin1 = LOW;
		inPin2 = HIGH;
	}
	if(motor == 0) {
		digitalWrite(AIN1, inPin1);
		digitalWrite(AIN2, inPin2);
		analogWrite(PWMA, speed);
	}
	if(motor == 1) {
		digitalWrite(BIN1, inPin1);
		digitalWrite(BIN2, inPin2);
		analogWrite(PWMB, speed);
	}
}