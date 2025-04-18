#include <QTRSensors.h>

constexpr float Kp = 0.2;
constexpr float Kd = 4;
constexpr float Ksr = 0;
constexpr int MaxSpeed = 255;
constexpr int BaseSpeed = 255;
constexpr int speedturn = 255;
constexpr int SensorCount = 8;

#define userButton 0
#define CENTER_POSITION 3500

int PWMA = 6, AIN1 = 4, AIN2 = 5;
int PWMB = 3, BIN1 = 1, BIN2 = 2;

class Motor {
  private:
    int pwmPin, in1, in2;

  public:
    Motor(int pwm, int pin1, int pin2) {
      pwmPin = pwm;
      in1 = pin1;
      in2 = pin2;
      pinMode(pwmPin, OUTPUT);
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
    }

    void move(int speed, bool forward) {
      digitalWrite(in1, forward ? HIGH : LOW);
      digitalWrite(in2, forward ? LOW : HIGH);
      analogWrite(pwmPin, speed);
    }

    void stop() {
      analogWrite(pwmPin, 0);
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }
};

class PIDController {
  private:
    float Kp, Kd;
    int lastError;

  public:
    PIDController(float p, float d) {
      Kp = p;
      Kd = d;
      lastError = 0;
    }

    int compute(int error) {
      int output = Kp * error + Kd * (error - lastError);
      lastError = error;
      return output;
    }
};

class LineFollower {
  private:
    QTRSensors qtr;
    uint16_t sensorValues[SensorCount];
    uint16_t smoothSensorValues[SensorCount];
    Motor* leftMotor;
    Motor* rightMotor;
    PIDController* pid;

  public:
    LineFollower(Motor* l, Motor* r, PIDController* p) {
      leftMotor = l;
      rightMotor = r;
      pid = p;
    }

    void setupSensors() {
      qtr.setTypeRC();
      qtr.setSensorPins((const uint8_t[]) {
        19, 18, 17, 16, 15, 14, 8, 7
      }, SensorCount);

      for (int i = 0; i < 100; i++) {
        if (i < 25 || i > 75) {
          leftMotor->move(60, true);
          rightMotor->move(60, false);
        } else {
          leftMotor->move(60, false);
          rightMotor->move(60, true);
        }
        qtr.calibrate();
        delay(10);
      }

      delay(100);
      while (digitalRead(userButton) == HIGH) {
        leftMotor->stop();
        rightMotor->stop();
      }
      delay(50);
      while (digitalRead(userButton) == LOW) {

      }
      delay(1000);
    }

    void smoothReadings() {
      for (int i = 0; i < SensorCount; i++) {
        smoothSensorValues[i] = (smoothSensorValues[i] + sensorValues[i]) / 2;
      }
    }

    void followLine() {
      uint16_t position = qtr.readLineBlack(sensorValues);
      smoothReadings();

      if (position == 0 || position == 7000) {
        leftMotor->stop();
        rightMotor->stop();
        delay(500);
        return;
      }

      if (position > 6700) {
        rightMotor->move(speedturn, false);
        leftMotor->move(speedturn, true);
        return;
      }

      if (position < 300) {
        rightMotor->move(speedturn, true);
        leftMotor->move(speedturn, false);
        return;
      }

      int error = position - CENTER_POSITION;
      int motorSpeed = pid->compute(error);

      double speedError = Ksr * (exp(1) * ((SensorCount * (SensorCount + 1)) / 2));

      int rightSpeed = BaseSpeed + motorSpeed - speedError;
      int leftSpeed = BaseSpeed - motorSpeed - speedError;

      rightSpeed = constrain(rightSpeed, 0, MaxSpeed);
      leftSpeed = constrain(leftSpeed, 0, MaxSpeed);

      rightMotor->move(rightSpeed, true);
      leftMotor->move(leftSpeed, true);
    }
};

Motor leftMotor(PWMA, AIN1, AIN2);
Motor rightMotor(PWMB, BIN1, BIN2);
PIDController pid(Kp, Kd);
LineFollower BotHudson(&leftMotor, &rightMotor, &pid);

void setup() {
  pinMode(userButton, INPUT_PULLUP);
  delay(200);
  BotHudson.setupSensors();
}

void loop() {
  BotHudson.followLine();
}
