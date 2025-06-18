/**
 * Simple sketch for testing motors of mBot ranger.
 * The routine is:
 * 1. make the motor go straight
 * 2. wait 1 second
 * 3. make the motor go backward
 * 4. wait 1 second
 */

#include <MeAuriga.h>

#define INTO_SEC(x) x * 1000

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

void setup()
{
}

void loop()
{
  int16_t pwm = 100;

  // Make it go straight
  Encoder_1.setMotorPwm(pwm);
  Encoder_2.setMotorPwm(pwm);
  delay(INTO_SEC(1));

  // Make it wait
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
  delay(INTO_SEC(1));

  // Make it go backwards
  Encoder_1.setMotorPwm(pwm * -1);
  Encoder_2.setMotorPwm(pwm * -1);
  delay(INTO_SEC(1));

  // Make it wait
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
  delay(INTO_SEC(1));
}
