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
#include "drivers/lsm6ds3_tools.h"


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


float accel[3] = {122.122, 125.125, 124.124};
uint16_t msg_len = 0;
state_msg_t msg;

void usart_init()
{
	/* Enable USART1 and GPIOA clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    /* Configure the GPIOs */
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Configure USART1 Tx (PA.09) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART1 Rx (PA.10) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure the USART1 */
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);

    /* Enable USART1 */
    USART_Cmd(USART1, ENABLE);
}

void msg_send(state_msg_t* msg)
{
	int i = -1;

	uint8_t* buff = (uint8_t*)msg;
	uint8_t len = sizeof(*msg) - 1;

//		USART_ClearFlag(USART1, USART_FLAG_TC);
		while (i < len) {
			while (!USART_GetFlagStatus(USART1, USART_FLAG_TXE));
			USART_SendData(USART1, buff[i++]);
			trace_printf("data %d %x\n", i, buff[i]);
		}
}

int main(int argc, char* argv[])
{

//	trace_printf("lsm init error %d", lsm6ds3_platform_init());

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

//	lsm6ds3_get_xl_data_g(accel);

//	trace_printf("data %f %f %f", accel[0], accel[1], accel[2]);
/*
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
*/
	usart_init();

	msg_len = 1;
	uint8_t tmp;

	do {
		tmp = 0;
		if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE))
		{
			USART_ClearFlag(USART1, USART_FLAG_RXNE);
			tmp = USART1->DR;

			trace_printf("tmp %d\n", tmp);

		};
	} while(tmp != 42);

	for (int i = 0; i < 3; i++)
	{
		msg.accel[i] = accel[i];
	}
	msg_send(&msg);

	return 0;
}

#pragma GCC diagnostic pop




// ----------------------------------------------------------------------------
