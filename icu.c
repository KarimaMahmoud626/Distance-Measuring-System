 /******************************************************************************
 *
 * Module: ICU
 *
 * File Name: icu.c
 *
 * Description: Source file for the AVR ICU driver
 *
 * Author: Karima Mahmoud
 *
 *******************************************************************************/

/*******************************************************************************
 *                         Header Files                                        *
 *******************************************************************************/
#include "icu.h"
#include "std_types.h"
#include "common_macros.h"/* To use macros like SET_BIT */
#include <avr/io.h>/* To use ICU/Timer1 registers */
#include <avr/interrupt.h>/* For ICU ISR */

/*******************************************************************************
 *                           Global Variables                                  *
 *******************************************************************************/
/* Global variable to hold the address of the call back function in the application */
static volatile void (*g_callBackPtr)(void) = NULL_PTR;

/*******************************************************************************
 *                       Interrupt Service Routines                            *
 *******************************************************************************/
ISR(TIMER1_CAPT_vect)
{
	if(g_callBackPtr != NULL_PTR)
	{
		/* Call the call back function in the application after the edge is detected */
		(*g_callBackPtr)();
	}
}

/*******************************************************************************
 *                      Functions Definitions                                  *
 *******************************************************************************/
/*[FUNCTION NAME]	: ICU_init
 *[DESCRIPTION]		:  Function to initialize the ICU driver
 * 	1. Set the required clock.
 * 	2. Set the required edge detection.
 * 	3. Enable the Input Capture Interrupt.
 * 	4. Initialize Timer1 Registers
 *[ARGUMENTS]		: const ICU_ConfigType * config_ptr
 *[RETURNS]			: void
 */
void ICU_init(const ICU_ConfigType *Config_Ptr)
{
	DDRD &= ~(1<<PD6);  /* Configure ICP1/PD6 as i/p pin */

	/* Timer1 always operates in normal mode */
	TCCR1A |= (1<<FOC1A) | (1<<FOC1B);

	/* Insert the required clock value in the first three bits CS10,CS11,CS12
	 * of TCCR1B register */
	TCCR1B = (TCCR1B&0xf8) | (Config_Ptr->clock);

	/* Insert the required edge type in ICES1 bit in TCCR1B register */
	TCCR1B = (TCCR1B&0xbf) | ((Config_Ptr->edge)<<6);

	/* Initial value of timer1 */
	TCNT1 = 0;

	/* Initial value of the input capture register */
	ICR1 = 0;

	/* Enable Input Capture interrupt */
	SET_BIT(TIMSK,TICIE1);
}

/*[FUNCTION NAME]	: ICU_setCallBack
 *[DESCRIPTION]		: Function to set the call back  function address
 *[ARGUMENTS]		: pointer to function
 *[RETURNS]			: void
 */
void ICU_setCallBack(void(*a_ptr)(void))
{
	/* Save the address of the Call back function in a global variable */
	g_callBackPtr = a_ptr;
}

/*[FUNCTION NAME]	: ICU_setEdgeDectectionType
 *[DESCRIPTION]		: Function to set the required edge detection.
 *[ARGUMENTS]		: edge of type ICU_EdgeType
 *[RETURNS]			: void
 */
void ICU_setEdgeDetectionType(const ICU_EdgeType a_edgeType)
{
	/* Insert the required edge type in ICES1 bit in TCCR1B register */
	TCCR1B = (TCCR1B&0xbf) | (a_edgeType<<6);
}

/*[FUNCTION NAME]	: ICU_getInputCaptureValue
 *[DESCRIPTION]		: Function to get the Timer1 Value when the input is captured
 *                    The value stored at Input Capture Register ICR1.
 *[ARGUMENTS]		: void
 *[RETURNS]			: uint16
 */
uint16 ICU_getInputCaptureValue(void)
{
	return ICR1;
}

/*[FUNCTION NAME]	: ICU_clearTimerValue
 *[DESCRIPTION]		: Function to clear the Timer1 Value to start count from ZERO
 *[ARGUMENTS]		: void
 *[RETURNS]			: void
 */
void ICU_clearTimerValue(void)
{
	TCNT1 = 0;
}

/*[FUNCTION NAME]	: ICU_clearTimerValue
 *[DESCRIPTION]		: Function to disable the Timer1 to stop the ICU Driver
 *[ARGUMENTS]		: void
 *[RETURNS]			: void
 */
void ICU_deInit(void)
{
	/* Clear all Timer1/ICU registers */
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0;
	ICR1 = 0;

	/* Disable the input capture interrupt */
	CLEAR_BIT(TIMSK,TICIE1);

	/* Reset the global pointer value */
	g_callBackPtr = NULL_PTR;
}

