#include <MeAuriga.h>
#include <Wire.h>

#ifdef __WIN32__
    #include <Math.h>
#else
    #include <math.h>
#endif

#define PWM 50
#define VISION_DEPTH 100.0
#define SAFETY_DISTANCE 5.0
#define STOPPING_DISTANCE 60.0
#define GYRO_THRESHOLD 1.5

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
    double angleDegrees;
};

struct Robot {
    int x;
    int y;
    double distanceCm;
    double angleDegrees;
    double speedCmPerSecond;
};

MeEncoderOnBoard RightMotor(SLOT1); // pwm < 0 va avanti
MeEncoderOnBoard LeftMotor(SLOT2); // pwm > 0 va avanti
MeGyro gyro(1, 0x69); // On Board external gryo sensor
MeUltrasonicSensor* sonic_sensor = NULL;
double initialGyroDegree = 0.0;

void busyWait(uint64_t ms)
{
    uint64_t startTime = millis();

    // Making sure it updates the gyro at least once
    do {
        gyro.update();
    } while (millis() - startTime < ms);
}

double getGyroDegrees()
{
    gyro.update();
    double val = gyro.getAngleZ() - initialGyroDegree;

    if (val > 360)
        val -= 360;

    if (val < 0)
        val += 360;

    return val;
}

void stop()
{
    RightMotor.setMotorPwm(0);
    LeftMotor.setMotorPwm(0);
}

void rotate(Rotation rotation)
{
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

void rotateTo(double degree)
{
    if (degree > getGyroDegrees())
        rotate(TO_THE_RIGHT);
    else
        rotate(TO_THE_LEFT);

    while (abs(degree - getGyroDegrees()) > GYRO_THRESHOLD) {
        gyro.update();
    }

    stop();
}

void move(Direction direction)
{
    switch (direction) {
    case FORWARD:
        RightMotor.setMotorPwm(-PWM);
        LeftMotor.setMotorPwm(PWM);
        while (sonic_sensor->distanceCm() >= SAFETY_DISTANCE) {}
        stop();
        break;
    case Direction::BACKWARD:
        RightMotor.setMotorPwm(PWM);
        LeftMotor.setMotorPwm(-PWM);
        break;
    }
}

void searchRobot(Rotation rotation)
{
    double initialDegree = getGyroDegrees();
    rotate(rotation);
    busyWait(200);

    do {
        gyro.update();
    } while (
        sonic_sensor->distanceCm() >= VISION_DEPTH
        && abs(initialDegree - getGyroDegrees()) > GYRO_THRESHOLD
    );

    stop();
}

void skipRobot(Rotation rotation)
{
    rotate(rotation);
    // TODO: handle when find no robot in 180 gradi
    while (sonic_sensor->distanceCm() < VISION_DEPTH) {
        gyro.update();
    }
    busyWait(100);
    stop();
}

/**
 * Calculates the Euclidean distance between two points
 */
double euclideanDistance(Robot& a, Robot& b)
{
    return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}

/**
 * The cosine law says that, given a triangle with sides a, b and c respectively at the opposite
 * of the vertices A, B, C, is it possible to find each internal angle as follows:
 * 
 * `A = arccos((b^2 + c^2 - a^2) / (2 * b * c))`
 * 
 * This function returns the internal angle (in degrees) opposite to side a
 */
double cosineLaw(double a, double b, double c)
{
    // Multiply by 180*pi to convert into degrees
    return acos((b*b + c*c - a*a) / (2 * b * c)) * (180 / M_PI);
}

/**
 * Given position of the other two robots, assuming self in (0, 0), returns the index of the robot
 * positioned at the angle >= 120 degrees:
 * - 0 -> Self
 * - 1 -> r1
 * - 2 -> r2
 */
int findLeader(Robot& robot1, Robot& robot2)
{
    Robot self = {
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
        Serial.println("No Robot is in angle > 120 degrees :(");
        stop();
        return -1;
    }

}

void step1DiscoverRobot(Robot& robot1, Robot& robot2)
{
    Serial.println("Step1");
    double angleRadiant = 0.0;

    Serial.println("Searching robot1");
    searchRobot(TO_THE_RIGHT);
    Serial.println("Fonud robot1");
    robot1.distanceCm = sonic_sensor->distanceCm();
    robot1.angleDegrees = getGyroDegrees();
    angleRadiant = robot1.angleDegrees * M_PI / 180.0;
    robot1.x = round(cos(angleRadiant * robot1.distanceCm));
    robot1.y = round(sin(angleRadiant * robot1.distanceCm));

    busyWait(1000);
    Serial.println("SkipingRobot");
    skipRobot(TO_THE_RIGHT);
    Serial.println("Skiped robot");
    busyWait(1000);

    Serial.println("Searching robot2");
    searchRobot(TO_THE_RIGHT);
    Serial.println("Found robot2");
    robot2.distanceCm = sonic_sensor->distanceCm();
    robot2.angleDegrees = getGyroDegrees();
    angleRadiant = robot2.angleDegrees * M_PI / 180.0;
    robot2.x = round(cos(angleRadiant * robot2.distanceCm));
    robot2.y = round(sin(angleRadiant * robot2.distanceCm));

    Serial.println("Finished Step1");
}

void step2GatherToLeader(Robot& leader)
{
    Serial.println("Step2");

    Serial.print("Rotating to: ");
    Serial.println(leader.angleDegrees);
    rotateTo(leader.angleDegrees);

    double initialDistance = sonic_sensor->distanceCm();
    uint64_t startTime = millis();

    Serial.print("Moving bitches");
    move(FORWARD);

    uint64_t elapsedSeconds = startTime - millis();
    leader.speedCmPerSecond = (initialDistance - SAFETY_DISTANCE) / elapsedSeconds;

    Serial.println("Finished Step2");
}

void setup()
{
    // Initialize serial with bound rate of 9600
    Serial.begin(9600);
    gyro.begin();
    sonic_sensor = new MeUltrasonicSensor(10);
    initialGyroDegree = getGyroDegrees();
}

void loop()
{
    Robot robot1;
    Robot robot2;
    step1DiscoverRobot(robot1, robot2);

    switch (findLeader(robot1, robot2)) {
    case 0:
        Serial.println("So io il leader");
        // TODO: implement case where I'm the leader
        break;
    case 1:
        Serial.println("robot1 è il leader");
        step2GatherToLeader(robot1);
        break;
    case 2:
        Serial.println("robot2 è il leader");
        step2GatherToLeader(robot2);
        break;
    default:
        Serial.println("No leader found");
        stop();
        break;
    }

    while (true) {}
}
