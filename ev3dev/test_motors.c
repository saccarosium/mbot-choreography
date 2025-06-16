#include <stdio.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"

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

// Helper functions
int min(int a, int b){
    return a > b? b: a;
}


// MAIN
int main( void )
{
	uint8_t l_motor_sn, r_motor_sn;

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
    // Waiting for initialization
	while ( ev3_tacho_init() < 1 ) Sleep( 1000 );

	printf( "*** ( EV3 ) Hello! ***\n" );

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

            printf("Setting speed to min(%d, %d)\n", speed, max_speed);

            set_tacho_speed_sp(l_motor_sn, min(speed, max_speed));
            set_tacho_speed_sp(r_motor_sn, min(speed, max_speed));

            // set_tacho_time_sp(l_motor_sn, 5000);
            // set_tacho_time_sp(r_motor_sn, 5000);
            // set_tacho_ramp_up_sp(l_motor_sn, 2000);
            // set_tacho_ramp_up_sp(r_motor_sn, 2000);
            // set_tacho_ramp_down_sp(l_motor_sn, 2000);
            // set_tacho_ramp_down_sp(r_motor_sn, 2000);
            

            printf("Assuming both are identical, running them in 5 seconds...\n");
            Sleep(5000);
            // Send the run command
            set_tacho_command_inx(l_motor_sn, TACHO_RUN_FOREVER);
            set_tacho_command_inx(r_motor_sn, TACHO_RUN_FOREVER);

            printf("Now sleeping for 5 seconds...\n");
            Sleep(5000);

            // Stop both motors
            set_tacho_command_inx(l_motor_sn, TACHO_STOP);
            set_tacho_command_inx(r_motor_sn, TACHO_STOP);
        }
    }


    ev3_uninit();
	printf( "*** ( EV3 ) Bye! ***\n" );

	return ( 0 );
}
