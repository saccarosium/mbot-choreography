#include <MeAuriga.h>
#include <Wire.h>
#include <Math.h>

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

MeGyro gyro(1,0x69); //On Board external gryo sensor


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

void rotateClockwiseBy(int degrees)
{
  int pwm = 40;
  double threshold = 0;
  gyro.update();
  double start = gyro.getAngleZ();
  double end = start + degrees; // 264

  if(end > 180){
    end -= 360; // -96
  }

  if(end < -180){
    end += 360;
  }

  if(degrees < 0){
    Serial.println("Rotating left");
    RightMotor.setMotorPwm(-pwm);
    LeftMotor.setMotorPwm(-pwm);
  } else {
    Serial.println("Rotating right");
    RightMotor.setMotorPwm(pwm);
    LeftMotor.setMotorPwm(pwm);
  }
  
  do{
    delay(10);
    gyro.update();
    Serial.print(" Z:");
    Serial.println(gyro.getAngleZ());
  }
  while(abs(end - gyro.getAngleZ()) > threshold);

  Serial.println("Stop rotation");
  stop();
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
  gyro.begin();
}

void loop()
{
  Serial.println("Hello");
  delay(2000);

  rotateClockwiseBy(90);
}
