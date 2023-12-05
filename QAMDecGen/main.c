/*
 * QAMDecGen.c
 *
 * Created: 20.03.2018 18:32:07
 * Author : Martin Burger
 */ 

//#include <avr/io.h>
#include "avr_compiler.h"
#include "pmic_driver.h"
#include "TC_driver.h"
#include "clksys_driver.h"
#include "sleepConfig.h"
#include "port_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "stack_macros.h"

#include "mem_check.h"

#include "init.h"
#include "utils.h"
#include "errorHandler.h"
#include "NHD0420Driver.h"

#include "qaminit.h"
#include "qamgen.h"
#include "qamdec.h"
#include "Display.h"


extern void vApplicationIdleHook( void );

void vApplicationIdleHook( void )
{	
	
}

int main(void)
{
	vInitClock();
	vInitDisplay();
	
	initDAC();
	initDACTimer();
	initGenDMA();
	initADC();
	initADCTimer();
	initDecDMA();
	
	xTaskCreate(vQuamGen, NULL, configMINIMAL_STACK_SIZE+500, NULL, 2, NULL);
	xTaskCreate(vQuamDec, NULL, configMINIMAL_STACK_SIZE+400, NULL, 1, NULL);
	xTaskCreate(vDisplay, NULL, configMINIMAL_STACK_SIZE+100, NULL, 3, NULL);

	vTaskStartScheduler();
	return 0;
}
