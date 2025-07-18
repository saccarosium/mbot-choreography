#include <MeAuriga.h>
#include <Wire.h>
#include <Math.h>

#define PWM 100
#define VISION_DEPTH 100.0

enum Direction {
    FORWARD,
    BACKWARD,
};

enum Rotation {
    TO_THE_LEFT,
    TO_THE_RIGHT,
};

MeEncoderOnBoard RightMotor(SLOT1); // pwm < 0 va avanti
MeEncoderOnBoard LeftMotor(SLOT2); // pwm > 0 va avanti
MeGyro gyro(1,0x69); // On Board external gryo sensor
MeUltrasonicSensor *sonic_sensor = NULL;

void stop()
{
    RightMotor.setMotorPwm(0);
    LeftMotor.setMotorPwm(0);
}

void rotate(Rotation rotation)
{
    switch (rotation) {
    case TO_THE_LEFT:
        RightMotor.setMotorPwm(PWM);
        LeftMotor.setMotorPwm(PWM);
        break;
    case TO_THE_RIGHT:
        RightMotor.setMotorPwm(-PWM);
        LeftMotor.setMotorPwm(-PWM);
        break;
    }
}

void move(Direction direction)
{
  switch (direction) {
  case FORWARD:
      RightMotor.setMotorPwm(-PWM);
      LeftMotor.setMotorPwm(PWM);
      break;
  case Direction::BACKWARD:
      RightMotor.setMotorPwm(PWM);
      LeftMotor.setMotorPwm(-PWM);
      break;
  }
}

void searchRobot(Rotation rotation)
{
    rotate(rotation);
    while (sonic_sensor->distanceCm() >= VISION_DEPTH) {};
    stop();
}

void setup()
{
    // Initialize serial with bound rate of 9600
    Serial.begin(9600);
    gyro.begin();
    sonic_sensor = new MeUltrasonicSensor(10);
}

void loop()
{
  Serial.println("Hello");
  delay(2000);

  rotateClockwiseBy(90);
}
