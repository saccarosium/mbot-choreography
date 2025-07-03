#include <MeAuriga.h>

#define PWM 100

enum class Direction {
  FORWARD,
  BACKWARD,
  RIGHT,
  LEFT,
};

enum class Rotation {
  CLOCKWISE,
  C_CLOCKWISE,
};

MeEncoderOnBoard RightMotor(SLOT1); // pwm < 0 va avanti
MeEncoderOnBoard LeftMotor(SLOT2); // pwm > 0 va avanti

void stop()
{
  RightMotor.setMotorPwm(0);
  LeftMotor.setMotorPwm(0);
}

void rotate(Rotation direction, int8_t slices)
{
  int8_t pwm = direction == Rotation::CLOCKWISE ? 50 : -50;
  for (int8_t i = 0; i < slices; ++i) {
    LeftMotor.setMotorPwm(pwm);
    RightMotor.setMotorPwm(pwm);
    delay(600);
    stop();
    delay(250);
  }
}

void move(Direction direction)
{
  switch (direction) {
    case Direction::FORWARD:
      RightMotor.setMotorPwm(-PWM);
      LeftMotor.setMotorPwm(PWM);
      break;
    case Direction::BACKWARD:
      RightMotor.setMotorPwm(PWM);
      LeftMotor.setMotorPwm(-PWM);
      break;
    case Direction::RIGHT:
      rotate(Rotation::CLOCKWISE, 2);
      RightMotor.setMotorPwm(-PWM);
      LeftMotor.setMotorPwm(PWM);
      break;
    case Direction::LEFT:
      rotate(Rotation::C_CLOCKWISE, 2);
      RightMotor.setMotorPwm(-PWM);
      LeftMotor.setMotorPwm(PWM);
      break;
    default:
      break;
  }
}

void setup()
{
  // Initialize serial with bound rate of 9600
  Serial.begin(9600);
}

void loop()
{
  Serial.println("Hello");
  delay(2000);
}
