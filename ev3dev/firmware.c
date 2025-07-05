#include <stdio.h>
#include <stdlib.h>
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

// Global variables
uint8_t l_motor_sn, r_motor_sn;
uint8_t ultrasonic_sn, gyro_sn;

int motorsSpeed;

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
	} else {
		if (!ev3_search_sensor(LEGO_EV3_GYRO, &gyro_sn, 0)) {
            printf("Gyro sensor should be on port in2, instead not found :(\n)");
        } else {
            printf("Found both ultrasonic (sn: %d) and gyro (sn: %d) sensors!\n", ultrasonic_sn, gyro_sn);
            
            // Continuous measurements for ultrasonic
            set_sensor_mode(ultrasonic_sn, "US-DIST-CM");

            // Gyro mode to measure angles
            set_sensor_mode(gyro_sn, "GYRO-ANG");
        }
	}

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

int getUsDistanceCm(){
	int val;
    get_sensor_value(0, ultrasonic_sn, &val);

    return val;
}

int getGyroDegrees(){
    int val;
    get_sensor_value(0, gyro_sn, &val);

    // Gyro outputs in range [-32768, 32767]
    while(val > 180){
        val -= 360;
    }
    while(val < -180){
        val += 360;
    }

    return val;
}

void rotateClockwiseBy(int degrees){
    int threshold = 5;
    int start = getGyroDegrees();
    int end = start + degrees;

    if(end > 180){
        end -= 360;
    }

    if(end < -180){
        end += 360;
    }

    if(degrees < 0){
        printf("Rotating left\n");
        move(LEFT);
    } else {
        printf("Rotating right\n");
        move(RIGHT);
    }
    
    do{
        Sleep(10);
        printf("Z: %d\n", getGyroDegrees());
    }
    while(abs(end - getGyroDegrees()) > threshold);

    printf("Stop rotation\n");
    stopMotors();
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

    // Measure data
    while(true){
        rotateClockwiseBy(90);
        Sleep(2000);
    }

    ev3_uninit();
	printf( "*** ( EV3 ) Bye! ***\n" );
	return ( 0 );
}
