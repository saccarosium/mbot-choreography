/**
 * Simple sketch for testing motors of mBot ranger.
 * The routine is:
 * 1. make the motor go straight
 * 2. wait 1 second
 * 3. make the motor go backward
 * 4. wait 1 second
 */

#include <MeAuriga.h>
#include <Wire.h>

/////////// Utility Functions ///////////

void wait()
{
  // 1 second
  delay(1000);
}

/////////// Motors //////////////////////

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

void test_motors()
{
  int16_t pwm = 100;

  // Make it go straight
  Encoder_1.setMotorPwm(pwm);
  Encoder_2.setMotorPwm(pwm);
  wait();
  // Make it wait
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
  wait();
  // Make it go backwards
  Encoder_1.setMotorPwm(pwm * -1);
  Encoder_2.setMotorPwm(pwm * -1);
  wait();
  // Make it wait
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
  wait();
}

/////////// LEDS ////////////////////////

#define ALLLEDS 0
// Auriga on-board light ring has 12 LEDs
#define LEDNUM  12
// on-board LED ring, at PORT0 (onboard)
MeRGBLed led(0, LEDNUM);

void test_rgb()
{
  // Pick a rgb value for every led
  float j, f, k;
  for (uint8_t t = 1; t < LEDNUM; ++t) {
    uint8_t red	= 64 * (1 + sin(t / 2.0 + j / 4.0));
    uint8_t green = 64 * (1 + sin(t / 1.0 + f / 9.0 + 2.1));
    uint8_t blue = 64 * (1 + sin(t / 3.0 + k / 14.0 + 4.2));
    led.setColorAt(t, red, green, blue);
  }
  led.show();
  wait();

  // Shutdown leds
  for (uint8_t t = 1; t < LEDNUM; ++t)
    led.setColorAt(t, 0, 0, 0);
  led.show();
  wait();
}

/////////// UltraSonic //////////////////

MeUltrasonicSensor *sonic_sensor = NULL;

void test_sonic_sensor()
{
  Serial.println("Starting monitoring sonic sensor for 5 seconds");
  for (int8_t i = 0; i < 5; ++i) {
    Serial.print("Value: ");
    Serial.println(sonic_sensor->distanceCm());
    delay(1000);
  }
}

/////////// Gyro ////////////////////////
MeGyro gyro(1,0x69);      //On Board external gryo sensor

void test_gyro()
{
  gyro.update();
  Serial.print("Z:");
  Serial.println(gyro.getAngleZ());
  delay(10);
}

/////////// Main ////////////////////////

void setup()
{
  // Initialize serial with bound rate of 9600
  Serial.begin(9600);
  // LED Ring controller is on Auriga D44/PWM
  led.setpin(44);
  // Init Ultrasonic Sensor in port 6
  sonic_sensor = new MeUltrasonicSensor(6);

  gyro.begin();
}

void loop()
{
  test_motors();
  test_rgb();
  test_sonic_sensor();
  test_gyro();
}
