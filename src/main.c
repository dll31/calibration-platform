#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "diag/Trace.h"
#include <stm32f10x.h>
//#include <usart.h>
//#include <timer.h>

#include "Timer.h"
//#include "BlinkLed.h"
#include "state.h"
#include "drivers/servo.h"


// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 3 / 4)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

int main(int argc, char* argv[])
{
	timer_start();
	timer_sleep(500);
	servo_init();
	motor_init();
	timer_sleep(500);

	timer_sleep(5000);

//	for (;;)
//	{
//		servo_next_pos();
//		timer_sleep(200);
//
//	}

	int motor_cnt = 12;
	for (int i = 0; i < motor_cnt; i++)
	{
		for (int k = 0; k < SERVO_STEPS; k++)
		{
			servo_next_pos();
			timer_sleep(200);
		}
		servo_start_pos();
		motor_next_pos();
	}


	return 0;
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
