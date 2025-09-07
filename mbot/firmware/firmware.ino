#include <MeAuriga.h>
#include <Wire.h>

#ifdef __WIN32__
#include <Math.h>
#else
#include <math.h>
#endif

#if 1
#define Print(msg) Serial.print(msg)
#define Println(msg) Serial.println(msg)
#else
#define Print(msg) (void*)0
#define Println(msg) (void*)0
#endif

#define PWM 50
#define GATHERING_STOP_DISTANCE_CM 20.0
#define GYRO_THRESHOLD 1.0


enum Direction {
  FORWARD,
  BACKWARD,
  };

enum Rotation {
  TO_THE_LEFT,
  TO_THE_RIGHT,
};

struct Point {
  double x;
  double y;
};

struct PolarPoint {
  double distanceCm;
  double angleDeg;
};

MeEncoderOnBoard RightMotor(SLOT1);  // pwm < 0 va avanti
MeEncoderOnBoard LeftMotor(SLOT2);   // pwm > 0 va avanti
MeGyro gyro(1, 0x69);                // On Board external gryo sensor
MeUltrasonicSensor* sonic_sensor = NULL;
double initialGyroDeg = 0.0;

double gatheringStopDistanceCm = 100.0f;

void busyWait(uint64_t ms) {
  uint64_t startTime = millis();

  // Making sure it updates the gyro at least once
  do {
    gyro.update();
  } while (millis() - startTime < ms);
}

void rotate(Rotation rotation) {
  switch (rotation) {
    case TO_THE_RIGHT:
      RightMotor.setMotorPwm(PWM);
      LeftMotor.setMotorPwm(PWM);
      break;
    case TO_THE_LEFT:
      RightMotor.setMotorPwm(-PWM);
      LeftMotor.setMotorPwm(-PWM);
      break;
  }
}

void stopMotors() {
  RightMotor.setMotorPwm(0);
  LeftMotor.setMotorPwm(0);
}

void move(Direction direction) {
  switch (direction) {
    case FORWARD:
      RightMotor.setMotorPwm(-PWM);
      LeftMotor.setMotorPwm(PWM);
      break;
    case BACKWARD:
      RightMotor.setMotorPwm(PWM);
      LeftMotor.setMotorPwm(-PWM);
      break;
  }
}

double getUsDistanceCm() {
  return sonic_sensor->distanceCm();
}

/**
 * Returns the angle always in range [0, 360], where 0 is the initial
 * orientation at the start of the program
 */
double getGyroDegrees() {
  gyro.update();

  // Subtract the initial offset of the gyro, so the measured angle
  // will always be zero at the robot's initial orientation
  double val = gyro.getAngleZ() - initialGyroDeg;

  while (val > 360)
    val -= 360;

  while (val < 0)
    val += 360;

  return val;
}

/**
 * Rotates at a specific angle. Input is assumed in range [0, 360]
 */
void rotateAtAngle(double degree) {
  Print("Rotating to ");
  Println(degree);

  if (degree > getGyroDegrees())
    rotate(TO_THE_RIGHT);
  else
    rotate(TO_THE_LEFT);

  while (abs(degree - getGyroDegrees()) > GYRO_THRESHOLD) {
    gyro.update();
  }

  stopMotors();
}

Point polarPointToPoint(const PolarPoint& polarPoint) {
  double angleRadial = polarPoint.angleDeg * M_PI / 180.0;

  return (Point){
    .x = round(cos(angleRadial) * polarPoint.distanceCm),
    .y = round(sin(angleRadial) * polarPoint.distanceCm),
  };
}

PolarPoint pointToPolarPoint(const Point& point) {
  double x = point.x;
  double y = point.y;

  return (PolarPoint){
    .distanceCm = round(sqrt(x * x + y * y)),
    .angleDeg = round(atan2(y, x) * 180.0 / M_PI),
  };
}


/**
 * Calculates the Euclidean distance between two points
 */
double euclideanDistance(Point& a, Point& b) {
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

/**
 * The cosine law says that, given a triangle with sides a, b and c respectively at the opposite
 * of the vertices A, B, C, is it possible to find each internal angle as follows:
 * 
 * `A = arccos((b^2 + c^2 - a^2) / (2 * b * c))`
 * 
 * This function returns the internal angle (in degrees) opposite to side a
 */
double cosineLaw(double a, double b, double c) {
  // Multiply by 180*pi to convert into degrees
  return acos((b * b + c * c - a * a) / (2 * b * c)) * (180 / M_PI);
}

/**
 * Given position of the other two robots, assuming self in (0, 0), returns the index of the robot
 * positioned at the angle >= 120 degrees:
 * - 0 -> Self
 * - 1 -> r1
 * - 2 -> r2
 */
int findLeader(Point& robot1, Point& robot2) {
  Point self = {
    .x = 0,
    .y = 0,
  };

  double oppositeToSelf = euclideanDistance(robot1, robot2);
  double oppositeToRobot1 = euclideanDistance(robot2, self);
  double oppositeToRobot2 = euclideanDistance(self, robot1);

  double angleSelf = cosineLaw(oppositeToSelf, oppositeToRobot1, oppositeToRobot2);
  double angleRobot1 = cosineLaw(oppositeToRobot1, oppositeToRobot2, oppositeToSelf);
  double angleRobot2 = cosineLaw(oppositeToRobot2, oppositeToSelf, oppositeToRobot1);

  if (angleSelf >= 120) {
    return 0;
  } else if (angleRobot1 >= 120) {
    return 1;
  } else if (angleRobot2 >= 120) {
    return 2;
  } else {
    Println("No robot is in angle > 120 degrees :(");
    stopMotors();
    return -1;
  }
}

/**
 * Moves towards the robot on that polar coordinates, stopping at some distance from it.
 * Additionally, calculates the average speed of the robot so that it can be used further.
 * The speed is expressed in centimeters/sec
 */
void step2GatherAtRobot(PolarPoint& other, double& measuredSpeedCmPerSecond) {
  Println("Step2");

  rotateAtAngle(other.angleDeg);

  double initialDistance = other.distanceCm;
  unsigned long startTime = millis();

  Print("startTime: ");
  Println(startTime);

  move(FORWARD);
  
  // Blocking until over SAFETY_DISTANCE
  while (sonic_sensor->distanceCm() >= GATHERING_STOP_DISTANCE_CM) {}
  stopMotors();

  unsigned long endTime = millis();
  Print("endTime: ");
  Println(endTime);

  double elapsedSeconds = (endTime - startTime) / 1000.0;

  Print("elapsedSeconds: ");
  Println(elapsedSeconds);

  measuredSpeedCmPerSecond = (initialDistance - GATHERING_STOP_DISTANCE_CM) / elapsedSeconds;

  Print("measuredSpeedCmPerSecond: ");
  Println(measuredSpeedCmPerSecond);

  Println("Finished Step2");
}

/**
 * Used by non-leaders, assuming the leader is in front of them.
 * Keeps measuring the distance until the leader moves away
 */
void step2WaitLeaderStartsMoving() {
  double initialDistance = sonic_sensor->distanceCm();

  // Since the ultrasonic sensor is not always reliable, the distance must
  // be this number of consecutive times above the threshold so that the
  // leader will be considered as moved away
  int checkDistanceFor = 3;

  do {
    busyWait(10);
    int distance = getUsDistanceCm();

    // If the measured distance is more than a certain value, it means the
    // leader has moved away
    if (distance > (initialDistance * 2))
      checkDistanceFor--;
    else
      checkDistanceFor = 3;

  } while (checkDistanceFor > 0);
}

/**
 * In case this robot is the one positioned at angle > 120 degrees, wait for
 * the  other two robots to gather
 */
void step2WaitForGathering(PolarPoint& r1, PolarPoint& r2) {
  Println("Waiting for gathering of robot 1");

  // Wait for robot 1
  rotateAtAngle(r1.angleDeg);
  int distance = 0;

  do {
    busyWait(10);
    distance = getUsDistanceCm();
  } while (distance > GATHERING_STOP_DISTANCE_CM);

  Println("Waiting for gathering of robot 2");

  // Wait for robot 2
  rotateAtAngle(r2.angleDeg);

  do {
    busyWait(10);
    distance = getUsDistanceCm();
  } while (distance > GATHERING_STOP_DISTANCE_CM);
}

Point calcMidpoint(Point& p1, Point& p2) {
  return (Point){
    .x = (p1.x + p2.x) / 2,
    .y = (p1.y + p2.y) / 2,
  };
}

/**
 * Given the leader robot and the other two robots, moves in the direction
 * linking the position of the leader robot and the middle point of the other
 * two robots
 */
void step3MoveInLine(Point& leaderCoord, Point& secondCoord, Point& thirdCoord, double speedCmPerSecond, bool isLeader) {
  Println("Calculating line to move");

  Point midPoint = calcMidpoint(secondCoord, thirdCoord);

  // Now, calculate the direction from the leader robot to the middle point
  Point movementOffset = {
    .x = midPoint.x - leaderCoord.x,
    .y = midPoint.y - leaderCoord.y,
  };

  PolarPoint movementPolarOffset = pointToPolarPoint(movementOffset);

  // The robot needs to move in the direction specified by this polar point
  rotateAtAngle(movementPolarOffset.angleDeg);

  double movementDistanceCm = 100.0;
  double movementTimeMs = (movementDistanceCm / speedCmPerSecond) * 1000.0;

  Print("movementTimeMs: ");
  Println(movementTimeMs);

  move(FORWARD);

  if (isLeader) {
    // The leader need to wait a few moments so the other robots can reach it
    double msToWait = 1000;  // TODO: Set correct waiting time
    stopMotors();
    busyWait(msToWait);
    move(FORWARD);
    busyWait(movementTimeMs - msToWait);
  } else {
    busyWait(movementTimeMs);
  }

  stopMotors();
}

/**
 * Used by the non-leaders to move away from the arrow formation 
 */
void step4MoveAway(PolarPoint& oldLeaderPosition, double speedCmPerSecond) {
  // The old leader position is the position the leader was before the gathering
  // So to move away, simply go to the opposite direction
  PolarPoint destination;
  destination.angleDeg = oldLeaderPosition.angleDeg - 180;
  destination.distanceCm = oldLeaderPosition.distanceCm;

  // Keep the angle in [0, 360] range
  if (destination.angleDeg < 0)
    destination.angleDeg += 360;

  if (destination.angleDeg >= 360)
    destination.angleDeg -= 360;

  rotateAtAngle(destination.angleDeg);

  double movementDistanceCm = destination.distanceCm;
  double movementTimeMs = (movementDistanceCm / speedCmPerSecond) * 1000.0;

  move(FORWARD);
  busyWait(movementTimeMs);
}

void setup() {
  // Initialize serial with bound rate of 9600
  Serial.begin(9600);
  gyro.begin();
  sonic_sensor = new MeUltrasonicSensor(10);
  initialGyroDeg = getGyroDegrees();
}

void loop() {
  PolarPoint robot1Polar;
  robot1Polar.angleDeg = 180;
  robot1Polar.distanceCm = 164;

  PolarPoint robot2Polar;
  robot2Polar.angleDeg = 192.3;
  robot2Polar.distanceCm = 100;

  Point robot1 = polarPointToPoint(robot1Polar);
  Print(robot1.x);
  Println(robot1.y);
  Point robot2 = polarPointToPoint(robot2Polar);
  Print(robot2.x);
  Println(robot2.y);

  Point self = {
    .x = 0,
    .y = 0,
  };

  int leader = findLeader(robot1, robot2);
  double speedCmPerSecond = 14.0;

  for (int iterations = 1; iterations > 0; --iterations) {
    switch (leader) {
      case 0:
        // other robots should move towards me
        step2WaitForGathering(robot1Polar, robot2Polar);
        busyWait(1000);  // This is just a delay between each phase

        // Here, the three robots have gathered
        Println("Gathering completed!");

        // Move in line
        step3MoveInLine(self, robot1, robot2, speedCmPerSecond, true);  // TODO: unknown speed

        // Here, robots have moved in line and are currently stopped
        Println("Line formation completed!");

        busyWait(1000);                                                  // Wait so the non-leaders can move forward and start the arrow formation
        step3MoveInLine(self, robot1, robot2, speedCmPerSecond, false);  // This time the leader should not wait anyone

        // Here, robots have moved in arrow formation and are still in this formation
        Println("Arrow formation completed!");

        // The leader should wait for a long time to allow the other two robots to reposition
        busyWait(15 * 1000);

        break;

      case 1:
        // gather at robot 1
        step2GatherAtRobot(robot1Polar, speedCmPerSecond);

        // Here, the three robots have gathered
        Println("Gathering completed!");

        // Move in line
        step2WaitLeaderStartsMoving();
        step3MoveInLine(robot1, robot2, self, speedCmPerSecond, false);

        // Here, robots have moved in line and are currently stopped
        Println("Line formation completed!");

        busyWait(10 * 1000); // TODO: Testing delay

        step3MoveInLine(robot1, robot2, self, speedCmPerSecond, false);

        // Here, robots have moved in arrow formation and are still in this formation
        Println("Arrow formation completed!");

        // The non-leaders should reposition
        step4MoveAway(robot1Polar, speedCmPerSecond);

        break;
      case 2:
        // gather at robot 2
        step2GatherAtRobot(robot2Polar, speedCmPerSecond);

        // Here, the three robots have gathered
        Println("Gathering completed!");

        // Move in line
        step2WaitLeaderStartsMoving();
        step3MoveInLine(robot2, robot1, self, speedCmPerSecond, false);

        // Here, robots have moved in line and are currently stopped
        Println("Line formation completed!");
        
        busyWait(10 * 1000); // TODO: Testing delay

        step3MoveInLine(robot2, robot1, self, speedCmPerSecond, false);

        // Here, robots have moved in arrow formation and are still in this formation
        Println("Arrow formation completed!");

        // The non-leaders should reposition
        step4MoveAway(robot2Polar, speedCmPerSecond);
        break;

      default:
        Println("This should never happen!");
        stopMotors();
        exit(1);
        break;
    }
  }

  // Stop the main loop to restart
  while (true) {};
}
