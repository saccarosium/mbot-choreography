#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"

// WIN32 /////////////////////////////////////////
#ifdef __WIN32__

#include <windows.h>

// UNIX //////////////////////////////////////////
#else

#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )

//////////////////////////////////////////////////
#endif

// Defining ports for left and right Tacho motors
#define L_MOTOR_PORT      OUTPUT_A
#define L_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define R_MOTOR_PORT      OUTPUT_B
#define R_MOTOR_EXT_PORT  EXT_PORT__NONE_

// Defining ports for sensors
#define ULTRASONIC_PORT         INPUT_1
#define GYRO_PORT       INPUT_2

typedef enum {
  FORWARD,
  BACKWARD,
  RIGHT,
  LEFT,
} Direction;

typedef struct {
    int x;
    int y;
} Point;

typedef struct {
    int distanceMm;
    int angleDeg;
} PolarPoint;

// Global variables
uint8_t l_motor_sn, r_motor_sn;
uint8_t ultrasonic_sn, gyro_sn;

int motorsSpeed;
int initialGyroDeg = 0;
int rotationThresholdDegrees = 5;

// Helper functions
int min(int a, int b){
    return a > b? b: a;
}

void initMotors(){
    printf("Initializing motors...\n");

    // Waiting for initialization
	while ( ev3_tacho_init() < 1 ) Sleep( 1000 );

    if(!ev3_search_tacho_plugged_in(L_MOTOR_PORT, L_MOTOR_EXT_PORT, &l_motor_sn, 0)){
        printf("Left motor should be on port outA, instead not found :(\n");
    }
    else{
        if(!ev3_search_tacho_plugged_in(R_MOTOR_PORT, R_MOTOR_EXT_PORT, &r_motor_sn, 0)){
            printf("Left motor should be on port outB, instead not found :(\n");
        }
        else{
            printf("Found both left (sn: %d) and right (sn: %d) motors!\n", l_motor_sn, r_motor_sn);

            int count_per_rot;
            get_tacho_count_per_rot(l_motor_sn, &count_per_rot);

            // Set speed to min(2 * 60 rpm, max_speed)
            int speed = 2 * 60 * count_per_rot;
            int max_speed;
            get_tacho_max_speed(l_motor_sn, &max_speed);

            printf("Setting speed to min(%d, %d / 2)\n", speed, max_speed);

            motorsSpeed = min(speed, max_speed / 2);

            // Setting stop action to hold the position
            // This allows to brake faster
            set_tacho_stop_action_inx(l_motor_sn, TACHO_HOLD);
            set_tacho_stop_action_inx(r_motor_sn, TACHO_HOLD);
        }
    }

    printf("Motors initialized\n");
}

void initSensors(){
    printf("Initializing sensors...\n");

    while(ev3_sensor_init() < 1) Sleep(1000);

    if (!ev3_search_sensor(LEGO_EV3_US, &ultrasonic_sn, 0)) {
		printf("Ultrasonic sensor should be on port in1, instead not found :(\n)");
        exit(1);
	} else {
		if (!ev3_search_sensor(LEGO_EV3_GYRO, &gyro_sn, 0)) {
            printf("Gyro sensor should be on port in2, instead not found :(\n)");
            exit(1);
        } else {
            printf("Found both ultrasonic (sn: %d) and gyro (sn: %d) sensors!\n", ultrasonic_sn, gyro_sn);
            
            // Continuous measurements for ultrasonic
            set_sensor_mode(ultrasonic_sn, "US-DIST-CM");

            // Gyro mode to measure angles
            set_sensor_mode(gyro_sn, "GYRO-ANG");
        }
	}

    // Set the initial gyro angle so to ignore it in further measurements
    get_sensor_value(0, gyro_sn, &initialGyroDeg);

    printf("Sensors initialized\n");
}

void stopMotors(){
    set_tacho_command_inx(l_motor_sn, TACHO_STOP);
    set_tacho_command_inx(r_motor_sn, TACHO_STOP);
}

void move(Direction direction){
    switch(direction){
        case FORWARD:
            set_tacho_speed_sp(l_motor_sn, -motorsSpeed);
            set_tacho_speed_sp(r_motor_sn, -motorsSpeed);
        case BACKWARD:
            set_tacho_speed_sp(l_motor_sn, motorsSpeed);
            set_tacho_speed_sp(r_motor_sn, motorsSpeed);
        case RIGHT:
            set_tacho_speed_sp(l_motor_sn, -motorsSpeed);
            set_tacho_speed_sp(r_motor_sn, motorsSpeed);
        case LEFT:
            set_tacho_speed_sp(l_motor_sn, motorsSpeed);
            set_tacho_speed_sp(r_motor_sn, -motorsSpeed);
    }
    set_tacho_command_inx(l_motor_sn, TACHO_RUN_FOREVER);
    set_tacho_command_inx(r_motor_sn, TACHO_RUN_FOREVER);
}

int getUsDistanceMm(){
	int val;
    get_sensor_value(0, ultrasonic_sn, &val);

    return val;
}

/**
 * Returns the angle always in range [0, 360], where 0 is the initial orientation at the start of the program
 */
int getGyroDegrees(){
    int val;
    get_sensor_value(0, gyro_sn, &val);

    // Subtract the initial offset of the gyro, so the measured angle
    // will always be zero at the robot's initial orientation
    val -= initialGyroDeg;

    // Gyro outputs in range [-32768, 32767]
    while(val > 360){
        val -= 360;
    }
    while(val < 0){
        val += 360;
    }

    return val;
}

/**
 * Rotates at a specific angle. Input is assumed in range [0, 360]
 */
void rotateAtAngle(int degrees){
    if(degrees > getGyroDegrees()){
        move(RIGHT);
    }
    else{
        move(LEFT);
    }

    do{
        Sleep(10);
    }
    while(abs(degrees - getGyroDegrees()) > rotationThresholdDegrees);
}

/**
 * Rotates and finds two robots
 */
void step1DiscoverRobots(PolarPoint *robot1, PolarPoint *robot2){
    int degrees = 180;
    int angleOffset = 25;
    int angleIgnore = 70;
    int thresholdDistance = 600;

    int start = getGyroDegrees();
    int end = start + degrees;

    // No need to clamp the values, we assume no rotation exceeds 360 degrees since its the initial one

    // Always rotates right
    printf("Rotating right\n");
    move(RIGHT);

    // Ignore any measument until these degrees of rotations are reached
    int ignoreRobotsUntil = start;

    // Number of robots found [0, 1]
    int robotsFound = 0;

    printf("Starting at degs: %d\n", start);
    
    do{
        Sleep(10);
        int distance = getUsDistanceMm();
        int rotation = getGyroDegrees();

        // printf("Z: %d\n", rotation);
        // printf("Dst: %d\n", distance);

        if(distance < thresholdDistance && ignoreRobotsUntil < rotation && robotsFound < 2){
            // The angle of the robot wrt initial orientation at start of the program, plus an offset
            int robotAngle = rotation + angleOffset;

            if(robotAngle > 360){
                robotAngle -= 360;
            }
            
            printf("Robot #%d found at (total) angle %d and distance %d\n", robotsFound, robotAngle, distance);
            
            ignoreRobotsUntil = rotation + angleIgnore;
            
            if(robotsFound == 0){
                robot1->distanceMm = distance;
                robot1->angleDeg = robotAngle;
            }
            else{
                if(robotsFound == 1){
                    robot2->distanceMm = distance;
                    robot2->angleDeg = robotAngle;
                }
            }
            
            robotsFound++;
            stopMotors();
            Sleep(2000);
            move(RIGHT);
        }
    }
    while(abs(end - getGyroDegrees()) > rotationThresholdDegrees);

    printf("Stop rotation\n");
    stopMotors();

    if(robotsFound < 2){
        printf("Robots not found :(");
        exit(1);
    }

    printf("Robot 1 found at %d with distance %d\n", robot1->angleDeg, robot1->distanceMm);
    printf("Robot 2 found at %d with distance %d\n", robot2->angleDeg, robot2->distanceMm);
}

/**
 * Converts polar coordinates into cartesian coordinate
 * Note that x axis is directed towards the orientation of the robot at the start of the program
 */
void polarPointToPoint(PolarPoint *polarPoint, Point *point){
    double angleRadiant = polarPoint->angleDeg * M_PI / 180.0;

    int x = (int)round(cos(angleRadiant) * polarPoint->distanceMm);
    int y = (int)round(sin(angleRadiant) * polarPoint->distanceMm);

    point->x = x;
    point->y = y;
}

/**
 * Converts a point to polar coordinates
 */
void pointToPolarPoint(Point *point, PolarPoint *polarPoint) {
    double x = (double)point->x;
    double y = (double)point->y;

    polarPoint->distanceMm = (int)round(sqrt(x * x + y * y));
    polarPoint->angleDeg = (int)round(atan2(y, x) * 180.0 / M_PI);
}


/**
 * Calculates the Euclidean distance between two points
 */
double euclideanDistance(Point *a, Point *b) {
    return sqrt((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
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
    return acos((b*b + c*c - a*a) / (2 * b * c)) * (180 / M_PI); // Multiply by 180*pi to convert into degrees
}


/**
 * Given position of the other two robots, assuming self in (0, 0), returns the index of the robot
 * positioned at the angle >= 120 degrees:
 * - 0 -> Self
 * - 1 -> r1
 * - 2 -> r2
 */
int calcRobotIn120Angle(Point *r1, Point *r2){
    Point self = {
        x: 0,
        y: 0
    }

    // Opposite to self
    double oppSelf = distance(r1, r2);
    // Opposite to r1
    double oppR1 = distance(r2, self);
    // Opposite to r2
    double oppR2 = distance(self, r1);

    double angleSelf = angle(oppSelf, oppR1, oppR2);
    double angleR1 = angle(oppR1, oppR2, oppSelf);
    double angleR2 = angle(oppR2, oppSelf, oppR1);

    if(angleSelf >= 120){
        return 0;
    }
    else {
        if(angleR1 >= 120){
            return 1;
        }
        else{
            if(angleR2 >= 120){
                return 2;
            }
            else{
                printf("No robot is in angle > 120 degrees :(");
                // Probably motors are already stopped, but do anyway
                stopMotors();
                exit(1);
            }
        }
    }
}

/**
 * Moves towards the robot on that polar coordinates, stopping at some distance from it.
 * Additionally, calculates the average speed of the robot so that it can be used further.
 * The speed is expressed in millimeters/sec
 */
void step2GatherAtRobot(PolarPoint *other, int *measuredSpeed){
    // Face the robot
    rotateAtAngle(other->angleDeg);

    // Move forward until a certain distance
    move(FORWARD);

    int intialDistance = getUsDistanceMm();
    int distance = intialDistance;
    int stopDistanceMm = 70;
    
    clock_t start = clock();

    do{
        Sleep(10);
        distance = getUsDistanceMm();
    }
    while(distance > stopDistanceMm);

    if(measuredSpeed != NULL){
        clock_t end = clock();

        double elapsedSeconds = (double)(end - start) / CLOCKS_PER_SEC;

        *measuredSpeed = (int)round((intialDistance - stopDistanceMm) / elapsedSeconds);
    }
}

/**
 * Given two points, calculates the middle point between them
 */
void calcMidpoint(Point *p1, Point *p2, Point *midpoint){
    midpoint->x = (p1->x + p2->x) / 2;
    midpoint->y = (p1->y + p2->y) / 2;
}

/**
 * Given the leader robot
 * and the other two robots, moves in the direction linking the position of the leader robot
 * and the middle point of the other two robots
 */
void step3MoveInLine(Point *leaderCoord, Point *secondCoord, Point *thirdCoord, int speedMmPerSecond){
    Point midPoint;
    calcMidpoint(&secondCoord, &thirdCoord, &midPoint);

    // Now, calculate the direction from the leader robot to the middle point
    Point movementOffset = {
        x: midPoint.x - leaderCoord.x,
        y: midPoint.y - leaderCoord.y
    }
    
    PolarPoint movementPolarOffset;
    pointToPolarPoint(&movementOffset, &movementPolarOffset);

    // The robot needs to move in the direction specified by this polar point
    rotateAtAngle(movementPolarOffset.angleDeg);
    move(FORWARD);

    int movementDistanceMm = 100;
    double movementTimeSeconds = movementDistanceMm / speedMmPerSecond;
    Sleep(movementTimeSeconds * 1000);
}

// MAIN
int main( void )
{

#ifndef __ARM_ARCH_4T__
	/* Disable auto-detection of the brick (you have to set the correct address below) */
	ev3_brick_addr = "192.168.0.204";

#endif
	if ( ev3_init() == -1 ) return ( 1 );

#ifndef __ARM_ARCH_4T__
	printf( "The EV3 brick auto-detection is DISABLED,\nwaiting %s online with plugged tacho...\n", ev3_brick_addr );

#else
	printf( "Waiting tacho is plugged...\n" );

#endif
	printf( "*** ( EV3 ) Hello! ***\n" );

    // Init devices
    initMotors(&l_motor_sn, &r_motor_sn);
    initSensors(&ultrasonic_sn, &gyro_sn);

    // Identify other robots polar coordinates
    PolarPoint robot1Polar, robot2Polar;
    step1DiscoverRobots(&robot1Polar, &robot2Polar);

    Point robot1, robot2;
    polarPointToPoint(&robot1Polar, &robot1);
    polarPointToPoint(&robot2Polar, &robot2);
    Point self = {
        x: 0,
        y: 0
    }

    printf("Robot1 x: %d, y: %d\n", robot1.x, robot1.y);
    printf("Robot2 x: %d, y: %d\n", robot2.x, robot2.y);

    // Calculate which robot is in 120 degrees angle
    int robotIn120Degrees = calcRobotIn120Angle(&robot1, &robot2);

    switch (robotIn120Degrees){
        case 0:
            // TODO: other robots should move towards me
            Sleep(1000); // TODO: choose waiting time

            // Move in line
            step3MoveInLine(&self, &robot1, &robot2);
            break;

        case 1:
            // gather at robot 1
            int speedMmPerSecond = 0;
            step2GatherAtRobot(&robot1Polar, &speedMmPerSecond);
            Sleep(1000); // TODO: choose waiting time

            // Move in line
            step3MoveInLine(&robot1, &robot2, &self);
            break;
        case 2:
            // gather at robot 2
            int speedMmPerSecond = 0;
            step2GatherAtRobot(&robot2Polar, &speedMmPerSecond);
            Sleep(1000); // TODO: choose waiting time

            // Move in line
            step3MoveInLine(&robot2, &robot1, &self);
            break;
        
        default:
            printf("This should never happen!");
            stopMotors();
            exit(1);
            break;
    }

    ev3_uninit();
	printf( "*** ( EV3 ) Bye! ***\n" );
	return ( 0 );
}
