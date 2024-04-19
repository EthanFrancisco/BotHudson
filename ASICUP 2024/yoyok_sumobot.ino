/*
	18/04/2023
	SumoBot with 3 SharpIR Sensors for Enemy Detection
*/
#include <math.h>
#include <SharpIR.h>
#include <QTRSensors.h>

QTRSensors qtr;
SharpIR sensor0(SharpIR::GP2Y0A21YK0F, 14);
SharpIR sensor1(SharpIR::GP2Y0A21YK0F, 15);
SharpIR sensor2(SharpIR::GP2Y0A21YK0F, 16);

const uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];

#define QTR_THRESHOLD 220
#define IR_THRESHOLD 17

// Speed Settings
#define speedTurn 190		// Default - 190
#define speedForward 165	// Default - 165
#define speedBackward 120	// Default - 120
#define speedCharge 255		// Default - 255

// Left Motor Pins
#define PWMA 3		// speedControl
#define AIN1 2		// pinDirection
#define AIN2 4		// pinDirection

// Right Motor Pins
#define PWMB 5		// speedControl
#define BIN1 6		// pinDirection
#define BIN2 7		// pinDirection

// Strategy Switches
#define leftSwitch 9	// Left Switch
#define rightSwitch 8	// Right Switch

void setup() {
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]) {
        18, 17
    }, SensorCount);
    Serial.begin(9600);
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(leftSwitch, INPUT_PULLUP);
    pinMode(rightSwitch, INPUT_PULLUP);
    delay(3500);
    if(digitalRead(leftSwitch) == LOW) {
        // turn left
        move(0, speedTurn, 0);
        move(1, speedTurn, 1);
        delay(150);
    }
    if(digitalRead(rightSwitch) == LOW) {
        // turn right
        move(0, speedTurn, 1);
        move(1, speedTurn, 0);
        delay(150);
    }
}

void loop() {
    qtr.read(sensorValues);
    int distance0 = sensor0.getDistance();
    int distance1 = sensor1.getDistance();
    int distance2 = sensor2.getDistance();
    if (sensorValues[0] < QTR_THRESHOLD) {
        // Leftmost Sensor Detected the Border
        move(1, speedBackward, 0);
        move(0, speedBackward, 0);
        delay(400);
        move(1, speedTurn, 1);
        move(0, speedTurn, 0);
        delay(275);
        move(1, speedForward, 1);
        move(0, speedForward, 1);
    } else if (sensorValues[1] < QTR_THRESHOLD) {
        // Rightmost Sensor Detected The Border
        move(1, speedBackward, 0);
        move(0, speedBackward, 0);
        delay(400);
        move(1, speedTurn, 0);
        move(0, speedTurn, 1);
        delay(275);
        move(1, speedForward, 1);
        move(0, speedForward, 1);
    } else {
        if((distance0 >= IR_THRESHOLD) && (distance1 >= IR_THRESHOLD) && (distance2 >= IR_THRESHOLD)) {
            move(1, speedForward, 1);
            move(0, speedForward, 1);
        } else {

            // Opponent in front center
            // Go straight in full speed
            if(distance1 <= IR_THRESHOLD) {
                move(1, speedCharge, 1);
                move(0, speedCharge, 1);
            }

            // Opponent in front left
            // Turn left
            if(distance2 <= IR_THRESHOLD) {
                move(1, speedCharge / 7, 1);
                move(0, speedCharge, 1);
            }

            // Opponent in front right
            // Turn right
            if(distance0 <= IR_THRESHOLD) {
                move(1, speedCharge, 1);
                move(0, speedCharge / 7, 1);
            }

            if((distance0 <= IR_THRESHOLD) && (distance1 <= IR_THRESHOLD) && (distance2 <= IR_THRESHOLD)) {
                move(1, speedCharge, 1);
                move(0, speedCharge, 1);
            }
        }
    }
}

// move() function declaration
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
